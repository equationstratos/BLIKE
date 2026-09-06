/**
 * What the two mass budgets actually cost.
 *
 * The firmware (Params.h) and the project's Gazebo model (reference/model.sdf)
 * do not carry the same masses: 110.7 g sits on the body in one and on the legs
 * in the other. Both cannot be right, and the LQR gain schedule was computed
 * for one of them. This measures the consequence rather than arguing about it.
 *
 * Run with:  node tools/compare-masses.mjs
 */

import { readFileSync } from 'node:fs';
import { POL } from '../src/pol.js';
import { LQRController } from '../src/lqr.js';
import { BODIES, WHEELS, LIMITS, DT } from '../src/params.js';

// --- read the Gazebo model's inertial data -------------------------------
const sdf = readFileSync(new URL('../reference/model.sdf', import.meta.url), 'utf8');

function linkInertial(name) {
  const block = sdf.match(new RegExp(`<link name="${name}">([\\s\\S]*?)</link>`))[1];
  const inertial = block.match(/<inertial>([\s\S]*?)<\/inertial>/)[1];
  const num = (tag) => parseFloat(inertial.match(new RegExp(`<${tag}>([-\\d.eE+]+)</${tag}>`))[1]);
  const pose = inertial.match(/<pose>([^<]+)<\/pose>/)[1].trim().split(/\s+/).map(Number);
  const [ixx, ixy, ixz, iyy, iyz, izz] = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'].map(num);
  return {
    m: num('mass'),
    c: pose.slice(0, 3),
    I: [ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz],
  };
}

// Same order as BODIES in params.js.
const SDF_LINKS = [
  'Mainbody', 'Thigh_Active_Right', 'Thigh_Active_Left',
  'Thigh_Passive_Right', 'Thigh_Passive_Left', 'Calf_Right', 'Calf_Left',
];
const sdfBodies = SDF_LINKS.map((name, i) => ({ ...linkInertial(name), p: BODIES[i].p, name: BODIES[i].name }));
const sdfWheels = { right: linkInertial('Wheel_Right'), left: linkInertial('Wheel_Left') };

// --- the two plants ------------------------------------------------------
const firmware = new POL();
const gazebo = new POL(sdfBodies, sdfWheels);

const deg = (r) => (r * 180) / Math.PI;
const pad = (s, n) => String(s).padStart(n);

console.log('\nEquivalent body, aggregated over the seven upper links');
console.log(`  mass          Params.h ${firmware.mB.toFixed(4)} kg    model.sdf ${gazebo.mB.toFixed(4)} kg    ${((gazebo.mB / firmware.mB - 1) * 100).toFixed(2)}%`);

console.log('\nBy commanded height: CoM offset, equilibrium pitch, pitch inertia');
console.log(`${pad('h (mm)', 7)} ${pad('p_bcom z (mm)', 26)} ${pad('theta_eq (deg)', 24)} ${pad('M(0,0)', 22)}`);
console.log(`${' '.repeat(7)} ${pad('firmware', 12)}${pad('gazebo', 12)}  ${pad('firmware', 11)}${pad('gazebo', 11)}  ${pad('firmware', 10)}${pad('gazebo', 10)}`);
for (const h of [0.07, 0.1, 0.13, 0.16, 0.2]) {
  const out = [firmware, gazebo].map((p) => {
    p.setHR(h, 0);
    p.computeComAndInertia();
    return { z: p.p_bcom[2] * 1000, eq: deg(p.thetaEquilibrium()), M: p.massMatrix(0)[0] };
  });
  console.log(
    `${pad((h * 1000).toFixed(0), 7)} ${pad(out[0].z.toFixed(2), 12)}${pad(out[1].z.toFixed(2), 12)}  ` +
    `${pad(out[0].eq.toFixed(3), 11)}${pad(out[1].eq.toFixed(3), 11)}  ` +
    `${pad(out[0].M.toExponential(3), 10)}${pad(out[1].M.toExponential(3), 10)}`,
  );
}

// --- the question that matters -------------------------------------------
// The gains were scheduled for one plant. Do they still hold up the other, and
// how much disturbance can each absorb before falling over?
function maxRecoverableTilt(plant, h) {
  const lqr = new LQRController();
  let lo = 0;
  let hi = 1.2;
  for (let iter = 0; iter < 24; iter++) {
    const tilt = (lo + hi) / 2;
    plant.setHR(h, 0);
    plant.computeComAndInertia();
    lqr.computeGain(h);
    let x = [tilt, 0, 0, 0];
    let ok = true;
    for (let k = 0; k < 2500; k++) {
      const u = lqr.computeInput([plant.thetaEquilibrium(), 0, 0, 0], x);
      const f = plant.derivative(x, u);
      if (!f) { ok = false; break; }
      for (let i = 0; i < 4; i++) x[i] += f[i] * DT;
      if (Math.abs(x[0]) > 1.05) { ok = false; break; }
    }
    if (ok && Math.abs(x[0] - plant.thetaEquilibrium()) < 0.02) lo = tilt; else hi = tilt;
  }
  return lo;
}

function maxRecoverableShove(plant, h) {
  const lqr = new LQRController();
  let lo = 0;
  let hi = 20;
  for (let iter = 0; iter < 24; iter++) {
    const kick = (lo + hi) / 2;
    plant.setHR(h, 0);
    plant.computeComAndInertia();
    lqr.computeGain(h);
    let x = [0, kick, 0, 0];
    let ok = true;
    for (let k = 0; k < 2500; k++) {
      const u = lqr.computeInput([plant.thetaEquilibrium(), 0, 0, 0], x);
      const f = plant.derivative(x, u);
      if (!f) { ok = false; break; }
      for (let i = 0; i < 4; i++) x[i] += f[i] * DT;
      if (Math.abs(x[0]) > 1.05) { ok = false; break; }
    }
    if (ok && Math.abs(x[0] - plant.thetaEquilibrium()) < 0.02) lo = kick; else hi = kick;
  }
  return lo;
}

console.log('\nStability margin, with the Params.h gain schedule driving each plant');
console.log(`${pad('h (mm)', 7)} ${pad('largest recoverable tilt (deg)', 34)} ${pad('largest recoverable shove (rad/s)', 37)}`);
console.log(`${' '.repeat(7)} ${pad('firmware', 12)}${pad('gazebo', 11)}${pad('change', 11)}  ${pad('firmware', 12)}${pad('gazebo', 11)}${pad('change', 11)}`);
for (const h of [0.07, 0.13, 0.2]) {
  const tf = maxRecoverableTilt(firmware, h);
  const tg = maxRecoverableTilt(gazebo, h);
  const sf = maxRecoverableShove(firmware, h);
  const sg = maxRecoverableShove(gazebo, h);
  console.log(
    `${pad((h * 1000).toFixed(0), 7)} ${pad(deg(tf).toFixed(2), 12)}${pad(deg(tg).toFixed(2), 11)}${pad(((tg / tf - 1) * 100).toFixed(1) + '%', 11)}  ` +
    `${pad(sf.toFixed(2), 12)}${pad(sg.toFixed(2), 11)}${pad(((sg / sf - 1) * 100).toFixed(1) + '%', 11)}`,
  );
}

// --- and the consequence that actually shows up on the robot --------------
// The controller aims the pitch at theta_eq, which it computes from the mass
// budget it was given. If that budget is the wrong one, the target is off by a
// fraction of a degree and the velocity loop spends itself holding a pitch the
// robot does not want - which comes out as a steady creep.
function drift(plant, reference, h) {
  const lqr = new LQRController();
  plant.setHR(h, 0);
  plant.computeComAndInertia();
  reference.setHR(h, 0);
  reference.computeComAndInertia();
  lqr.computeGain(h);

  const target = reference.thetaEquilibrium(); // what the controller believes
  let x = [plant.thetaEquilibrium(), 0, 0, 0];
  for (let k = 0; k < 6000; k++) {
    const u = lqr.computeInput([target, 0, 0, 0], x);
    const f = plant.derivative(x, u);
    if (!f) return NaN;
    for (let i = 0; i < 4; i++) x[i] += f[i] * DT;
  }
  return x[2];
}

console.log('\nSteady creep when the controller is given the wrong budget');
console.log(`${pad('h (mm)', 7)} ${pad('theta_eq gap (deg)', 20)} ${pad('creep, gazebo robot (m/s)', 27)} ${pad('creep, firmware robot (m/s)', 29)}`);
for (const h of [0.07, 0.13, 0.2]) {
  firmware.setHR(h, 0); firmware.computeComAndInertia();
  gazebo.setHR(h, 0); gazebo.computeComAndInertia();
  const gap = deg(gazebo.thetaEquilibrium() - firmware.thetaEquilibrium());
  // robot has gazebo masses, controller aims with the firmware's theta_eq
  const a = drift(gazebo, firmware, h);
  // and the mirror case
  const b = drift(firmware, gazebo, h);
  console.log(`${pad((h * 1000).toFixed(0), 7)} ${pad(gap.toFixed(3), 20)} ${pad(a.toFixed(3), 27)} ${pad(b.toFixed(3), 29)}`);
}
console.log();
