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
   * Nonlinear effects vector (POL::calculate_nle).
   *
   * `gyro` below is the velocity-coupling coefficient the firmware writes out
   * twice: it is the psi_dot^2 factor of the pitch equation and, doubled, the
   * psi_dot*theta_dot factor of the yaw equation.
   *
   * The firmware's yaw equation closes that second parenthesis early, so its
   * last three terms land straight in nle(2) instead of being scaled by
   * psi_dot*theta_dot. That leaves a constant yaw acceleration (~2.3 rad/s^2
   * at rest, no torque applied), which is not what the hardware does. It is an
   * operator-precedence slip rather than a modelling choice - the terms are
   * exactly twice the matching ones in the pitch equation - so we group them
   * as intended.
   */
  nle(theta, thetaDot, v, psiDot) {
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
    const td2 = thetaDot * thetaDot;
    const pd2 = psiDot * psiDot;

    // Wheel gyroscopic coupling, shared by the pitch and yaw equations.
    const wheelCoupling =
      (ct * (at(LW, 1, 0) + at(RW, 1, 0)) + st * (at(LW, 2, 1) + at(RW, 2, 1))) / R;

    const gyro =
      at(IB, 2, 2) + at(LW, 2, 2) + at(RW, 2, 2) -
      mB *
        (h * pc[0] + pc[0] * pc[2] -
          ct2 * (h * pc[0] * 2 + pc[0] * pc[2] * 2) -
          sc * (h * pc[2] * 2 + h2 - pc[0] * pc[0] + pc[2] * pc[2])) -
      ct2 * (at(IB, 2, 2) * 2 + at(LW, 2, 2) * 2 + at(RW, 2, 2) * 2) +
      sc *
        (at(IB, 0, 0) - at(IB, 2, 2) + at(LW, 0, 0) + at(RW, 0, 0) -
          at(LW, 2, 2) - at(RW, 2, 2)) +
      (L * (ct * (at(LW, 1, 0) - at(RW, 1, 0)) + st * (at(LW, 2, 1) - at(RW, 2, 1)))) / R;

    const n = [0, 0, 0];

    n[0] =
      -pd2 * gyro -
      GRAVITY * mB * (st * (h + pc[2]) + pc[0] * ct) +
      psiDot * v * wheelCoupling;

    n[1] =
      -mB * td2 * (st * (h + pc[2]) + pc[0] * ct) -
      psiDot * thetaDot * wheelCoupling;

    n[2] =
      -td2 *
        (-mB * (st * (pc[1] * (h + pc[2])) + pc[0] * pc[1] * ct) +
          at(IB, 1, 1) * ct + at(IB, 2, 2) * st) +
      psiDot * thetaDot * 2 * gyro -
      thetaDot * v * wheelCoupling;

    return n;
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
