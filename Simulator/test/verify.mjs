/**
 * Numerical checks for the ported model.
 *
 * Run with:  node test/verify.mjs
 *
 * These back the specific claims the README makes. Nothing here needs a
 * browser or a network; it is plain Node with no dependencies.
 */

import { POL } from '../src/pol.js';
import { LQRController } from '../src/lqr.js';
import { EKF } from '../src/ekf.js';
import { measure } from '../src/sensors.js';
import { GEOM, DT } from '../src/params.js';
import { eye, mat, mul, transpose, add } from '../src/linalg.js';

let failures = 0;
const check = (name, ok, detail) => {
  console.log(`${ok ? '  pass' : '  FAIL'}  ${name}${detail ? `  (${detail})` : ''}`);
  if (!ok) failures++;
};
const deg = (r) => (r * 180) / Math.PI;

// Deterministic noise, so the thresholds below mean something.
let seed = 20250903;
Math.random = () => {
  seed = (seed * 1103515245 + 12345) & 0x7fffffff;
  return seed / 0x7fffffff;
};

// ---------------------------------------------------------------------------
console.log('\nLeg kinematics — the five-bar closes at every height');
{
  const p = new POL();
  let worstCoupler = 0;
  let worstCalf = 0;
  for (let h = GEOM.l5 + 0.07; h <= 0.2 + 1e-9; h += 0.005) {
    p.setHR(h, 0);
    p.computeComAndInertia();
    for (const side of [0, 1]) {
      const q = p.legPoints(side);
      const planar = (a, b) => Math.hypot(a[0] - b[0], a[2] - b[2]);
      worstCoupler = Math.max(worstCoupler, Math.abs(planar(q.D, q.C) - GEOM.l3));
      worstCalf = Math.max(
        worstCalf,
        Math.abs(planar(q.D, q.E) - Math.hypot(GEOM.l4, GEOM.l5)),
      );
    }
  }
  check('knee coupler |DC| holds l3', worstCoupler < 1e-9, `max error ${worstCoupler.toExponential(2)} m`);
  // The firmware's IK and FK are not perfectly consistent with each other, and
  // that discrepancy is inherited deliberately: it is what the hardware
  // commands. Peak is 2.30 mm (1.7% of the calf) around h = 114 mm.
  check('calf |DE| within 2.5 mm of sqrt(l4^2+l5^2)', worstCalf < 2.5e-3, `max error ${(worstCalf * 1000).toFixed(2)} mm`);
}

// ---------------------------------------------------------------------------
console.log('\nJacobians — analytic vs central finite differences');
{
  const p = new POL();
  const cases = [
    { h: 0.2, x: [0.05, 0.3, 0.4, 0.2], u: [0.2, -0.15] },
    { h: 0.07, x: [-0.12, -0.8, -0.6, 0.9], u: [-0.4, 0.3] },
    { h: 0.17, x: [0.3, 1.2, 0.9, -1.1], u: [0.7, 0.7] },
  ];
  let worst = 0;
  for (const c of cases) {
    p.setHR(c.h, 0);
    p.computeComAndInertia();
    const A = p.stateJacobian(c.x, c.u);
    for (let j = 0; j < 4; j++) {
      const e = Math.max(1e-6, Math.abs(c.x[j]) * 1e-6);
      const xp = c.x.slice();
      const xm = c.x.slice();
      xp[j] += e;
      xm[j] -= e;
      const fp = p.derivative(xp, c.u);
      const fm = p.derivative(xm, c.u);
      for (let i = 0; i < 4; i++) {
        const num = (fp[i] - fm[i]) / (2 * e);
        worst = Math.max(worst, Math.abs(A[i * 4 + j] - num) / Math.max(1, Math.abs(num)));
      }
    }
  }
  check('df/dx matches finite differences', worst < 1e-6, `worst relative error ${worst.toExponential(2)}`);
}

// ---------------------------------------------------------------------------
console.log('\nCovariance propagation — why the discrete Jacobian is needed');
{
  const p = new POL();
  p.setHR(0.2, 0);
  p.computeComAndInertia();
  const fx = p.stateJacobian([0.05, 0.2, 0.1, 0.05], [0.1, -0.1]);
  const Q = mat(4, 4);
  Q.d[5] = Q.d[10] = Q.d[15] = 1;
  const tr = (M) => M.d[0] + M.d[5] + M.d[10] + M.d[15];

  const run = (F) => {
    let P = eye(4);
    for (let k = 0; k < 40; k++) P = add(mul(mul(F, P), transpose(F)), Q);
    return tr(P);
  };
  const Phi = eye(4);
  for (let i = 0; i < 16; i++) Phi.d[i] += fx[i] * DT;

  const literal = run(mat(4, 4, fx)); // the firmware's continuous Jacobian
  const correct = run(Phi);
  check('firmware form diverges over 40 steps', literal > 1e40, `tr(P) = ${literal.toExponential(2)}`);
  check('discrete form stays bounded', correct < 1e6, `tr(P) = ${correct.toExponential(2)}`);
}

// ---------------------------------------------------------------------------
console.log('\nClosed loop — LQR on the EKF estimate');
{
  const scenarios = [
    { name: 'balance at full height', h: 0.2, vd: 0, wd: 0, th0: 0.1, noise: 1 },
    { name: 'drive and turn', h: 0.15, vd: 0.6, wd: 0.8, th0: 0.05, noise: 1 },
    { name: 'low stance', h: 0.07, vd: 0, wd: 0, th0: 0.08, noise: 1 },
    // At 3x noise the pitch estimate carries ~1.5 deg of error, and the LQR's
    // theta gain is roughly eight times its velocity gain, so a steady-state
    // velocity offset of a few tenths of a m/s is the expected consequence
    // rather than a fault. Here we only require that it stays upright.
    { name: 'heavy sensor noise', h: 0.2, vd: 0.3, wd: 0, th0: 0.05, noise: 3, uprightOnly: true },
  ];
  for (const s of scenarios) {
    const pol = new POL();
    const lqr = new LQRController();
    const ekf = new EKF(pol);
    pol.setHR(s.h, 0);
    pol.computeComAndInertia();
    lqr.computeGain(s.h);
    ekf.reset();

    let x = [s.th0, 0, 0, 0];
    let u = [0, 0];
    let fell = false;
    let errSum = 0;
    let n = 0;

    for (let k = 0; k < 3000; k++) {
      const xdot = pol.derivative(x, u);
      if (!xdot) { fell = true; break; }
      const xh = ekf.step(measure(x, xdot, pol, s.noise), u, DT);
      if (!xh) { fell = true; break; }
      if (k > 1500) { errSum += Math.abs(xh[0] - x[0]); n++; }
      u = lqr.computeInput([pol.thetaEquilibrium(), 0, s.vd, s.wd], xh);
      const f = pol.derivative(x, u);
      for (let i = 0; i < 4; i++) x[i] += f[i] * DT;
      if (Math.abs(x[0]) > 1.05) { fell = true; break; }
    }
    const meanErr = deg(errSum / n);
    const tracks = s.uprightOnly
      ? !fell && meanErr < 3
      : !fell && Math.abs(x[2] - s.vd) < 0.15 && Math.abs(x[3] - s.wd) < 0.15;
    check(
      s.name,
      tracks,
      fell ? 'fell over' : `v=${x[2].toFixed(3)} ψ̇=${x[3].toFixed(3)} mean |θ̂-θ|=${meanErr.toFixed(3)}°`,
    );
  }
}

// ---------------------------------------------------------------------------
console.log('\nEstimator bias — isolating the dropped acceleration terms');
{
  const run = (withAccel, vd) => {
    const pol = new POL();
    const lqr = new LQRController();
    const ekf = new EKF(pol);
    pol.setHR(0.2, 0);
    pol.computeComAndInertia();
    lqr.computeGain(0.2);
    ekf.reset();
    let x = [0, 0, 0, 0];
    let u = [0, 0];
    let sum = 0;
    let n = 0;
    for (let k = 0; k < 3000; k++) {
      const xdot = pol.derivative(x, u);
      const xh = ekf.step(measure(x, withAccel ? xdot : null, pol, 0), u, DT);
      if (!xh) return NaN;
      if (k > 1500) { sum += Math.abs(xh[0] - x[0]); n++; }
      u = lqr.computeInput([pol.thetaEquilibrium(), 0, vd, 0], xh);
      const f = pol.derivative(x, u);
      for (let i = 0; i < 4; i++) x[i] += f[i] * DT;
    }
    return deg(sum / n);
  };
  const matched = run(false, 0.8);
  const withAcc = run(true, 0.8);
  const atRest = run(true, 0);
  check('no bias when the measurement matches the filter', matched < 1e-3, `${matched.toFixed(4)}°`);
  check('no bias at rest', atRest < 1e-3, `${atRest.toFixed(4)}°`);
  check('bias appears only under acceleration', withAcc > 0.1 && withAcc < 1, `${withAcc.toFixed(3)}° at 0.8 m/s`);
}

// ---------------------------------------------------------------------------
// The project also ships a Gazebo model (reference/model.sdf) built from the
// CAD. Its link poses are an account of the same linkage written down
// independently of the firmware this simulator was ported from, so agreeing
// with it is real evidence rather than a self-consistency check.
console.log('\nCross-check against the Gazebo model in reference/model.sdf');
{
  const p = new POL();
  const hipAt = (h) => { p.setHR(h, 0); p.computeComAndInertia(); return p.thetaHips[0]; };

  // The SDF is posed at theta_B = 0: its Calf_Right sits exactly at
  // (-(a + l2), -0.102, b). Solve for the height that produces that.
  let lo = 0.07;
  let hi = 0.2;
  for (let i = 0; i < 200; i++) {
    const m = (lo + hi) / 2;
    if (hipAt(lo) * hipAt(m) <= 0) hi = m; else lo = m;
  }
  const h = (lo + hi) / 2;
  p.setHR(h, 0);
  p.computeComAndInertia();
  const q = p.legPoints(0);

  // Angles and the calf origin should match to numerical precision.
  const exact = [
    ['theta_A', p.thetaA[0], 0.02631897651],
    ['theta_K', p.thetaK[0], 0.767322043371783],
    ['calf origin x', q.C[0], -0.141951905284],
    ['calf origin z', q.C[2], 0.0375],
  ];
  let worst = 0;
  for (const [, a, b] of exact) worst = Math.max(worst, Math.abs(a - b));
  check('link angles and calf origin match the SDF', worst < 1e-8, `worst ${worst.toExponential(2)}`);

  // The wheel is the interesting one. The SDF puts its axle 1.8 mm forward of
  // straight-down, because the real A->E vector is not vertical. The reduced
  // model assumes it is - but it gets the *distance* right, which is what the
  // dynamics actually use. This pins both halves of that statement.
  const sdfWheel = [-0.001800801039, -0.123, -0.08655913097823];
  const distSdf = Math.hypot(sdfWheel[0], sdfWheel[2]);
  check('|A->E| matches the SDF', Math.abs(distSdf - h) < 2e-5, `${distSdf.toFixed(6)} m vs h = ${h.toFixed(6)} m`);
  const forward = Math.abs(q.E[0] - sdfWheel[0]);
  check(
    'wheel forward offset is the known ~1.8 mm simplification',
    forward > 1.5e-3 && forward < 2.5e-3,
    `${(forward * 1000).toFixed(2)} mm, i.e. ${((Math.atan2(forward, h) * 180) / Math.PI).toFixed(2)}deg off vertical`,
  );
}

console.log(`\n${failures ? `${failures} check(s) FAILED` : 'all checks passed'}\n`);
process.exit(failures ? 1 : 0);
