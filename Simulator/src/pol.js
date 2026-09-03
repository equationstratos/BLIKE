/**
 * POL - "Pendulum On Legs" model of the POW robot.
 *
 * A faithful JavaScript port of ESP32/WBR_Control/POL.h from
 * SeungbinOh/Pow_WBR_Project. It provides:
 *
 *   - the five-bar leg inverse kinematics (height/roll  ->  hip servo angles),
 *   - the forward kinematics that place every link,
 *   - the aggregation of the seven upper bodies into a single equivalent body
 *     (CoM offset p_bcom and inertia I_B_B seen from the wheel axle),
 *   - the reduced 4-state dynamics  M(theta) qdd + nle = B u.
 *
 * State  x  = [theta, theta_dot, v, psi_dot]
 *   theta     body pitch (rad, 0 = upright)
 *   v         forward velocity of the wheel axle (m/s)
 *   psi_dot   yaw rate (rad/s)
 * Input  u  = [tau_RW, tau_LW]  (N*m)
 *
 * Frames. The body frame has +x forward, +y to the left, +z up, with its
 * origin on the hip axis. The leg linkage is planar; inside the kinematics we
 * use planar coordinates (u, w) that map to the body frame as
 * (x, z) = (-u, w), which is the convention baked into POL.h's p_vecs.
 *
 * Two deliberate deviations from the firmware, both bug fixes:
 *   - p_bcom and I_B_B are reset before each aggregation (the firmware
 *     accumulates into them across calls),
 *   - phi is kept in degrees on the object instead of being overwritten with
 *     radians on every inverse-kinematics call.
 */

import { BODIES, WHEELS, GEOM, LIMITS, GRAVITY } from './params.js';
import { mat3, vec3, parallelAxis, clamp } from './mat.js';

const acosSafe = (x) => Math.acos(clamp(x, -1, 1));

export class POL {
  constructor() {
    const { a, b, l1, l2, l3, l4, l5, L, R } = GEOM;
    Object.assign(this, { a, b, l1, l2, l3, l4, l5, L, R });

    this.sqrFwd = a * a + b * b + l1 * l1 + l2 * l2 - l3 * l3;
    this.angleEDF = Math.atan(l5 / l4);
    this.AB = Math.hypot(a, b);

    this.mBodies = BODIES.map((x) => x.m);
    this.mB = this.mBodies.reduce((s, m) => s + m, 0);
    this.mRW = WHEELS.right.m;
    this.mLW = WHEELS.left.m;
    this.I_RW = WHEELS.right.I;
    this.I_LW = WHEELS.left.I;

    // Input matrix B (3x2), rows [theta, v, psi], columns [tau_RW, tau_LW].
    this.B = [
      [1, -1],
      [-1 / R, 1 / R],
      [-L / R, -L / R],
    ];

    this.h = LIMITS.HEIGHT_MAX;
    this.phiDeg = 0;

    // Filled in by the kinematics / aggregation.
    this.thetaHips = [0, 0]; // [right, left], as commanded to the hip servos
    this.thetaB = [0, 0];
    this.thetaA = [0, 0];
    this.thetaK = [0, 0];
    this.hSide = [this.h, this.h];
    this.phiApplied = 0; // rad, after saturation
    this.p_bcom = [0, 0, 0];
    this.I_B_B = mat3.zero();
    this.singular = false;
  }

  setHR(h, phiDeg) {
    this.h = clamp(h, LIMITS.HEIGHT_MIN, LIMITS.HEIGHT_MAX);
    this.phiDeg = clamp(phiDeg, LIMITS.PHI_MIN, LIMITS.PHI_MAX);
  }

  /**
   * Height and roll -> hip servo angles.
   * Mirrors POL::solve_inverse_kinematics, including the roll saturation that
   * keeps both legs inside [HEIGHT_MIN, HEIGHT_MAX].
   */
  solveInverseKinematics() {
    const { a, b, l1, l2, l3, l4, l5, L } = this;
    const h = this.h;

    let phi = (this.phiDeg * Math.PI) / 180;
    const phiMax = Math.min(
      Math.atan((h - LIMITS.HEIGHT_MIN) / L),
      Math.atan((LIMITS.HEIGHT_MAX - h) / L),
    );
    phi = clamp(phi, -phiMax, phiMax);
    this.phiApplied = phi;

    // Right leg shortens as the body rolls positive, left leg lengthens.
    const hs = [h - L * Math.tan(phi), h + L * Math.tan(phi)];
    this.hSide = hs;

    const DE = Math.hypot(l4, l5);
    for (let i = 0; i < 2; i++) {
      const hv = hs[i];
      const angleADE = acosSafe((l1 * l1 + DE * DE - hv * hv) / (2 * l1 * DE));
      const angleADC = Math.PI - (angleADE + this.angleEDF);
      const AC = Math.sqrt(l1 * l1 + l3 * l3 - 2 * l1 * l3 * Math.cos(angleADC));
      const angleABC = acosSafe((this.AB * this.AB + l2 * l2 - AC * AC) / (2 * this.AB * l2));
      this.thetaHips[i] = (5 * Math.PI) / 6 - angleABC;
    }
    this.thetaHips[1] = -this.thetaHips[1]; // the left servo is mirrored
    return this.thetaHips;
  }

  /**
   * Hip servo angles -> full link placement.
   * Mirrors POL::solve_forward_kinematics.
   */
  solveForwardKinematics() {
    const { a, b, l1, l2 } = this;
    this.thetaB = [-this.thetaHips[0], this.thetaHips[1]];

    for (let i = 0; i < 2; i++) {
      const cB = Math.cos(this.thetaB[i]);
      const sB = Math.sin(this.thetaB[i]);

      const cu = a + l2 * cB; // C, in planar coordinates
      const cw = b + l2 * sB;
      const dist = Math.hypot(cu, cw);
      const num = this.sqrFwd + 2 * l2 * (a * cB + b * sB);
      const ratio = num / (2 * l1) / dist;

      const alpha = Math.atan2(cw, cu);
      this.thetaA[i] = -acosSafe(ratio) + alpha;

      const du = l1 * Math.cos(this.thetaA[i]); // D, in planar coordinates
      const dw = l1 * Math.sin(this.thetaA[i]);
      this.thetaK[i] = Math.atan2(cw - dw, cu - du);
    }
    return { thetaA: this.thetaA, thetaB: this.thetaB, thetaK: this.thetaK };
  }

  /**
   * Joint positions of one leg in the body frame, for rendering.
   * side: 0 = right (-y), 1 = left (+y).
   */
  legPoints(side) {
    const { a, b, l1, l2, L } = this;
    const sign = side === 0 ? -1 : 1;
    const cB = Math.cos(this.thetaB[side]);
    const sB = Math.sin(this.thetaB[side]);
    const cA = Math.cos(this.thetaA[side]);
    const sA = Math.sin(this.thetaA[side]);
    // planar (u, w) -> body (x, z) = (-u, w)
    return {
      A: [0, sign * 0.081, 0], // passive thigh pivot (body origin plane)
      B: [-a, sign * 0.086, b], // hip servo output
      C: [-(a + l2 * cB), sign * 0.102, b + l2 * sB], // knee coupler / calf origin
      D: [-l1 * cA, sign * 0.102, l1 * sA], // knee
      E: [0, sign * L, -this.hSide[side]], // wheel axle
    };
  }

  /**
   * Aggregate the seven upper bodies into one equivalent body.
   * Mirrors POL::calculate_com_and_inertia (with the accumulation bug fixed).
   */
  computeComAndInertia() {
    this.solveInverseKinematics();
    this.solveForwardKinematics();

    const { a, b, l2 } = this;
    const R = new Array(7);
    const p = new Array(7);
    R[0] = mat3.identity();
    for (let i = 0; i < 5; i++) p[i] = BODIES[i].p;
    R[1] = mat3.rotY(this.thetaB[0]);
    R[2] = mat3.rotY(this.thetaB[1]);
    R[3] = mat3.rotY(this.thetaA[0]);
    R[4] = mat3.rotY(this.thetaA[1]);
    R[5] = mat3.rotY(this.thetaK[0]);
    R[6] = mat3.rotY(this.thetaK[1]);
    for (const side of [0, 1]) {
      const cB = Math.cos(this.thetaB[side]);
      const sB = Math.sin(this.thetaB[side]);
      p[5 + side] = [-(a + l2 * cB), side === 0 ? -0.102 : 0.102, b + l2 * sB];
    }

    const rB = new Array(7);
    const com = [0, 0, 0];
    for (let i = 0; i < 7; i++) {
      rB[i] = vec3.add(p[i], mat3.apply(R[i], BODIES[i].c));
      com[0] += this.mBodies[i] * rB[i][0];
      com[1] += this.mBodies[i] * rB[i][1];
      com[2] += this.mBodies[i] * rB[i][2];
    }
    this.p_bcom = vec3.scale(com, 1 / this.mB);

    const I = mat3.zero();
    for (let i = 0; i < 7; i++) {
      const rot = mat3.mulT(mat3.mul(R[i], BODIES[i].I), R[i]);
      mat3.addScaled(I, rot, 1);
      mat3.addScaled(I, parallelAxis(vec3.sub(rB[i], this.p_bcom), this.mBodies[i]), 1);
    }
    this.I_B_B = I;
    return { p_bcom: this.p_bcom, I_B_B: I };
  }

  /** Equilibrium pitch: the angle that puts the CoM over the wheel contact. */
  thetaEquilibrium() {
    return Math.atan(-this.p_bcom[0] / (this.h + this.p_bcom[2]));
  }

  /** Mass matrix M(theta) (POL::calculate_M). */
  massMatrix(theta) {
    const IB = this.I_B_B;
    const LW = this.I_LW;
    const RW = this.I_RW;
    const at = (A, i, j) => A[i * 3 + j];
    const pc = this.p_bcom;
    const { L, R } = this;
    const mB = this.mB;
    const h = this.h;

    const ct = Math.cos(theta);
    const st = Math.sin(theta);
    const ct2 = ct * ct;
    const sc = st * ct;
    const h2 = h * h;

    const M = new Float64Array(9);
    M[0] =
      at(IB, 1, 1) + mB * (h2 + pc[0] * pc[0] + pc[2] * pc[2]) + 2 * h * mB * pc[2];

    M[3] = mB * (-st * pc[0] + ct * (h + pc[2]));
    M[1] = M[3];

    M[6] =
      ct * (at(IB, 2, 1) - mB * pc[1] * (h + pc[2])) -
      st * (at(IB, 1, 0) - mB * pc[0] * pc[1]);
    M[2] = M[6];

    M[4] = mB + this.mLW + this.mRW + (at(LW, 1, 1) + at(RW, 1, 1)) / (R * R);

    M[7] =
      -mB * pc[1] -
      L * (this.mLW - this.mRW) -
      (L * (at(LW, 1, 1) - at(RW, 1, 1))) / (R * R) +
      (ct * (at(LW, 2, 1) + at(RW, 2, 1))) / R -
      (st * (at(LW, 1, 0) + at(RW, 1, 0))) / R;
    M[5] = M[7];

    M[8] =
      at(IB, 0, 0) + at(LW, 0, 0) + at(RW, 0, 0) +
      (L * L * (at(LW, 1, 1) + at(RW, 1, 1))) / (R * R) +
      L * L * (this.mLW + this.mRW) +
      mB * (h2 + pc[1] * pc[1] + pc[2] * pc[2]) -
      ct * ((at(LW, 2, 1) * L * 2) / R - (at(RW, 2, 1) * L * 2) / R) +
      st * ((at(LW, 1, 0) * L * 2) / R - (at(RW, 1, 0) * L * 2) / R) -
      ct2 *
        (at(IB, 0, 0) - at(IB, 2, 2) + at(LW, 0, 0) + at(RW, 0, 0) -
          at(LW, 2, 2) - at(RW, 2, 2) +
          mB * (h * pc[2] * 2 + h2 - pc[0] * pc[0] + pc[2] * pc[2])) -
      sc *
        (at(IB, 2, 0) * 2 + at(LW, 2, 0) * 2 + at(RW, 2, 0) * 2 -
          mB * (pc[0] * (h + pc[2])) * 2) +
      h * mB * pc[2] * 2;
    return M;
  }

  /**
   * Coefficients shared by the nonlinear effects vector and its Jacobians,
   * together with their theta derivatives. Deriving both from one place is
   * what keeps `nle` and `jacobians` consistent with each other - the firmware
   * writes each of these out by hand several times over, which is where its
   * index slips crept in (see the note on `jacobians`).
   */
  modelCoefficients(theta) {
    const IB = this.I_B_B;
    const LW = this.I_LW;
    const RW = this.I_RW;
    const at = (A, i, j) => A[i * 3 + j];
    const pc = this.p_bcom;
    const { L, R } = this;
    const mB = this.mB;
    const h = this.h;

    const ct = Math.cos(theta);
    const st = Math.sin(theta);
    const ct2 = ct * ct;
    const sc = st * ct;
    const c2t = 2 * ct2 - 1; // cos(2 theta), the derivative factor of sin*cos

    // Yaw-axis inertia sums, and the pitch-roll coupling group.
    const G0 = at(IB, 2, 2) + at(LW, 2, 2) + at(RW, 2, 2);
    const X =
      at(IB, 0, 0) - at(IB, 2, 2) + at(LW, 0, 0) + at(RW, 0, 0) -
      at(LW, 2, 2) - at(RW, 2, 2);
    const P1 = 2 * pc[0] * (h + pc[2]);
    const P2 = 2 * h * pc[2] + h * h - pc[0] * pc[0] + pc[2] * pc[2];

    // Differential wheel term.
    const dl = at(LW, 1, 0) - at(RW, 1, 0);
    const dr = at(LW, 2, 1) - at(RW, 2, 1);
    const Lterm = (L * (ct * dl + st * dr)) / R;
    const dLterm = (L * (-st * dl + ct * dr)) / R;

    // Velocity-coupling coefficient: the psi_dot^2 factor of the pitch
    // equation, and (doubled) the psi_dot*theta_dot factor of the yaw one.
    const gyro =
      G0 -
      mB * (h * pc[0] + pc[0] * pc[2] - ct2 * P1 - sc * P2) -
      2 * ct2 * G0 +
      sc * X +
      Lterm;
    const dGyro = -mB * (2 * sc * P1 - c2t * P2) + 4 * sc * G0 + c2t * X + dLterm;

    // Wheel gyroscopic coupling.
    const sl = at(LW, 1, 0) + at(RW, 1, 0);
    const sr = at(LW, 2, 1) + at(RW, 2, 1);
    const W = (ct * sl + st * sr) / R;
    const dW = (-st * sl + ct * sr) / R;

    // Body pendulum term, and the yaw equation's theta_dot^2 coefficient.
    const pend = st * (h + pc[2]) + pc[0] * ct;
    const dPend = ct * (h + pc[2]) - pc[0] * st;
    const A =
      -mB * (st * (pc[1] * (h + pc[2])) + pc[0] * pc[1] * ct) +
      at(IB, 1, 1) * ct + at(IB, 2, 2) * st;
    const dA =
      -mB * (ct * (pc[1] * (h + pc[2])) - pc[0] * pc[1] * st) -
      at(IB, 1, 1) * st + at(IB, 2, 2) * ct;

    return { ct, st, ct2, sc, c2t, gyro, dGyro, W, dW, pend, dPend, A, dA, X, P1, P2, G0 };
  }

  /**
   * Nonlinear effects vector (POL::calculate_nle).
   *
   * The firmware's yaw equation closes a parenthesis early, so its last three
   * terms land straight in nle(2) instead of being scaled by psi_dot*theta_dot.
   * That leaves a constant yaw acceleration (~2.3 rad/s^2 at rest, no torque
   * applied), which is not what the hardware does. Its own derivative,
   * calculate_dnle_dtheta, groups those terms inside the psi_dot*theta_dot
   * factor - so the firmware disagrees with itself, and it is nle(2) that is
   * wrong. We group them as intended.
   */
  nle(theta, thetaDot, v, psiDot) {
    const c = this.modelCoefficients(theta);
    const td2 = thetaDot * thetaDot;
    const pd2 = psiDot * psiDot;

    return [
      -pd2 * c.gyro - GRAVITY * this.mB * c.pend + psiDot * v * c.W,
      -this.mB * td2 * c.pend - psiDot * thetaDot * c.W,
      -td2 * c.A + psiDot * thetaDot * 2 * c.gyro - thetaDot * v * c.W,
    ];
  }

  /**
   * Jacobians of the model about the current state: dM/dtheta, dnle/dtheta and
   * dnle/dqdot, as POL::calculate_dM_dtheta, calculate_dnle_dtheta and
   * calculate_dnle_dqdot compute them. Only the extended Kalman filter needs
   * these; the plain simulation does not.
   *
   * The firmware's three Jacobian routines carry a family of index slips from
   * the MATLAB derivation they were transcribed from: I(2,2) written as I(2,0),
   * I(2,1) as I(2,0), I(1,1) as I(1,0). Counting the inertia references makes
   * it plain - the Jacobians use I_B_B(2,0) twelve times to I_B_B(2,2)'s seven,
   * while the functions they differentiate use (2,2) fifteen times. Checked
   * against a finite difference of `massMatrix` and `nle`, the transcribed
   * versions are wrong by up to a factor of seven on the yaw entries, which no
   * Kalman filter survives. These are derived from the shared coefficients
   * above instead, and agree with finite differences to ~1e-9.
   */
  jacobians(theta, thetaDot, v, psiDot) {
    const IB = this.I_B_B;
    const LW = this.I_LW;
    const RW = this.I_RW;
    const at = (A, i, j) => A[i * 3 + j];
    const pc = this.p_bcom;
    const { L, R } = this;
    const mB = this.mB;
    const h = this.h;

    const c = this.modelCoefficients(theta);
    const { ct, st, sc, c2t } = c;
    const td2 = thetaDot * thetaDot;
    const pd2 = psiDot * psiDot;

    // --- dM/dtheta -------------------------------------------------------
    const dM = new Float64Array(9);
    // M(0,0) has no theta dependence.
    dM[3] = mB * (-ct * pc[0] - st * (h + pc[2]));
    dM[1] = dM[3];
    dM[6] =
      -st * (at(IB, 2, 1) - mB * pc[1] * (h + pc[2])) -
      ct * (at(IB, 1, 0) - mB * pc[0] * pc[1]);
    dM[2] = dM[6];
    dM[7] = -c.W; // d/dtheta of the wheel term in M(2,1)
    dM[5] = dM[7];

    const Xm = c.X + mB * c.P2;
    const Ym = 2 * (at(IB, 2, 0) + at(LW, 2, 0) + at(RW, 2, 0)) - mB * c.P1;
    // The first term is the doubled differential wheel term inside M(2,2).
    dM[8] =
      (2 * L * (ct * (at(LW, 1, 0) - at(RW, 1, 0)) + st * (at(LW, 2, 1) - at(RW, 2, 1)))) / R +
      2 * sc * Xm -
      c2t * Ym;

    // --- dnle/dtheta -----------------------------------------------------
    const dnle_dtheta = [
      -pd2 * c.dGyro - GRAVITY * mB * c.dPend + psiDot * v * c.dW,
      -mB * td2 * c.dPend - psiDot * thetaDot * c.dW,
      -td2 * c.dA + psiDot * thetaDot * 2 * c.dGyro - thetaDot * v * c.dW,
    ];

    // --- dnle/dqdot, rows = equations, columns = [theta_dot, v, psi_dot] ---
    const dq = new Float64Array(9);
    dq[0] = 0;
    dq[1] = psiDot * c.W;
    dq[2] = -2 * psiDot * c.gyro + v * c.W;

    dq[3] = -2 * mB * thetaDot * c.pend - psiDot * c.W;
    dq[4] = 0;
    dq[5] = -thetaDot * c.W;

    dq[6] = -2 * thetaDot * c.A + 2 * psiDot * c.gyro - v * c.W;
    dq[7] = -thetaDot * c.W;
    dq[8] = 2 * thetaDot * c.gyro;

    return { dM_dtheta: dM, dnle_dtheta, dnle_dqdot: dq };
  }

  /**
   * Continuous-time state Jacobian df/dx (POL::calculate_fx), about (x, u),
   * as a row-major 4x4. Returns null on a singular mass matrix.
   */
  stateJacobian(x, u) {
    const [theta, thetaDot, v, psiDot] = x;
    const M = this.massMatrix(theta);
    const Minv = mat3.inverse(M);
    if (!Minv) return null;

    const n = this.nle(theta, thetaDot, v, psiDot);
    const { dM_dtheta, dnle_dtheta, dnle_dqdot } = this.jacobians(theta, thetaDot, v, psiDot);

    const rhs = [0, 0, 0];
    for (let i = 0; i < 3; i++) {
      rhs[i] = -n[i] + this.B[i][0] * u[0] + this.B[i][1] * u[1];
    }
    const qdd = mat3.apply(Minv, rhs);

    // first column: -M^-1 (dM/dtheta * qdd + dnle/dtheta)
    const inner = mat3.apply(dM_dtheta, qdd).map((z, i) => -(z + dnle_dtheta[i]));
    const col0 = mat3.apply(Minv, inner);
    // lower-right block: -M^-1 dnle/dqdot
    const block = mat3.mul(Minv, dnle_dqdot);

    const fx = new Float64Array(16); // row-major 4x4
    fx[1] = 1; // d(theta)/d(theta_dot)
    for (let i = 0; i < 3; i++) {
      fx[(i + 1) * 4] = col0[i];
      for (let j = 0; j < 3; j++) fx[(i + 1) * 4 + (j + 1)] = -block[i * 3 + j];
    }
    return fx;
  }

  /**
   * State derivative xdot = f(x, u).
   * Returns null when the mass matrix is singular, which is how the firmware
   * detects a diverging model and cuts the motors.
   */
  derivative(x, u) {
    const [theta, thetaDot, v, psiDot] = x;
    const M = this.massMatrix(theta);
    const Minv = mat3.inverse(M);
    if (!Minv) {
      this.singular = true;
      return null;
    }
    this.singular = false;

    const n = this.nle(theta, thetaDot, v, psiDot);
    const rhs = [0, 0, 0];
    for (let i = 0; i < 3; i++) {
      rhs[i] = -n[i] + this.B[i][0] * u[0] + this.B[i][1] * u[1];
    }
    const qdd = mat3.apply(Minv, rhs);
    return [thetaDot, qdd[0], qdd[1], qdd[2]];
  }
}
