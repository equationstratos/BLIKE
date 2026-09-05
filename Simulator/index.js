/**
 * POW wheeled bipedal robot - 3D simulator and log visualiser.
 *
 * Two modes share one scene:
 *   simulate - the firmware's own model runs live. The gain-scheduled LQR from
 *              VYBController.h closes the loop around the POL dynamics at the
 *              firmware's 125 Hz, and you fly it with the keyboard.
 *   replay   - the same robot is driven from a flight log recorded on the real
 *              hardware, so you can watch what actually happened.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

import { POL } from './src/pol.js';
import { LQRController } from './src/lqr.js';
import { DT, LIMITS, HIP_SERVO } from './src/params.js';
import { clamp } from './src/mat.js';
import { EKF } from './src/ekf.js';
import { measure } from './src/sensors.js';
import { Robot3D, buildScene } from './src/robot3d.js';
import { loadLinkMeshes, LINK_MESHES } from './src/meshes.js';
import { Hud } from './src/hud.js';
import { LogPlayer, LOGS } from './src/logplayer.js';

// --------------------------------------------------------------------------
// scene
// --------------------------------------------------------------------------
const canvasHost = document.getElementById('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
canvasHost.append(renderer.domElement);

const scene = new THREE.Scene();
// Everything below is authored with +z up, the convention POL.h uses.
const world = new THREE.Group();
world.rotation.x = -Math.PI / 2;
scene.add(world);
const { key: keyLight, followLight } = buildScene(scene, world);

const camera = new THREE.PerspectiveCamera(45, 1, 0.02, 120);
// Slightly off the centreline: dead astern hides the leg linkage behind the chassis.
const CHASE = { distance: 1.45, rise: 0.5, azimuth: 0.42 };
camera.position.set(-CHASE.distance, CHASE.rise, 0);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 0.3;
controls.maxDistance = 8;
controls.maxPolarAngle = Math.PI / 2 - 0.02;

const robot = new Robot3D();
world.add(robot.group);

function resize() {
  const w = canvasHost.clientWidth;
  const h = canvasHost.clientHeight;
  renderer.setSize(w, h); // updateStyle on: the canvas must lay out at CSS size
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', () => {
  resize();
  hud.resize();
});

// --------------------------------------------------------------------------
// simulation state
// --------------------------------------------------------------------------
const pol = new POL();
const lqr = new LQRController();
// The estimator runs on the same POL instance as the plant, so there is no
// parameter mismatch between model and truth: what the filter has to cope with
// is sensor noise plus the observation model's dropped acceleration terms.
const ekf = new EKF(pol);

const sim = {
  x: [0, 0, 0, 0], // [theta, theta_dot, v, psi_dot]
  pose: { x: 0, y: 0, psi: 0 },
  hCmd: LIMITS.HEIGHT_MAX,
  hTarget: LIMITS.HEIGHT_MAX,
  phiCmd: 0,
  u: [0, 0],
  fallen: false,
  fallenAt: 0,
  time: 0,
  xHat: [0, 0, 0, 0], // EKF estimate
  estError: [0, 0, 0, 0], // estimate minus truth, compared at the same instant
  useEstimate: true, // close the loop on the estimate, as the firmware does
  noiseScale: 1,
};

function resetSim() {
  sim.x = [0, 0, 0, 0];
  sim.pose = { x: 0, y: 0, psi: 0 };
  sim.hCmd = LIMITS.HEIGHT_MAX;
  sim.hTarget = LIMITS.HEIGHT_MAX;
  sim.u = [0, 0];
  sim.fallen = false;
  sim.time = 0;
  sim.xHat = [0, 0, 0, 0];
  sim.estError = [0, 0, 0, 0];
  ekf.reset();
  banner.classList.remove('show');
}

const keys = new Set();
const HEIGHT_RATE = 0.28; // m/s, roughly what the hip servos manage

/** One firmware control period: reference, gain, torque, integration. */
function stepSim(dt) {
  // --- operator commands -------------------------------------------------
  const crouch = keys.has('space');
  const target = crouch ? LIMITS.HEIGHT_MIN : sim.hTarget;
  sim.hCmd += clamp(target - sim.hCmd, -HEIGHT_RATE * dt, HEIGHT_RATE * dt);

  let vCmd = 0;
  if (keys.has('w')) vCmd += LIMITS.VEL_MAX;
  if (keys.has('s')) vCmd -= LIMITS.VEL_MAX;
  let yawCmd = 0;
  if (keys.has('a')) yawCmd += LIMITS.YAW_MAX;
  if (keys.has('d')) yawCmd -= LIMITS.YAW_MAX;

  if (sim.fallen) {
    vCmd = 0;
    yawCmd = 0;
  }

  // --- model update, exactly the order WBR_Control.ino uses ---------------
  pol.setHR(sim.hCmd, sim.phiCmd);
  pol.computeComAndInertia();

  // Sense, then estimate. The measurement is generated from the true state and
  // its true accelerations; the filter's observation model drops those, so it
  // faces the same modelling error it does on the robot.
  const xdotTrue = pol.derivative(sim.x, sim.u);
  if (xdotTrue) {
    const z = measure(sim.x, xdotTrue, pol, sim.noiseScale);
    const xhat = ekf.step(z, sim.u, dt);
    if (xhat) {
      sim.xHat = xhat.slice();
      // Compare against the truth the measurement was taken from, before this
      // step integrates it forward - otherwise the readout shows one step of
      // state change (0.5 deg at 1 rad/s) as if it were estimation error.
      sim.estError = xhat.map((e, i) => e - sim.x[i]);
    } else if (!sim.fallen) {
      // The firmware cuts the motors when the estimator faults; do the same.
      sim.fallen = true;
      sim.fallenAt = sim.time;
    }
  }

  const xd = [pol.thetaEquilibrium(), 0, vCmd, yawCmd];
  lqr.computeGain(sim.hCmd);
  const measured = sim.useEstimate ? sim.xHat : sim.x;
  sim.u = sim.fallen ? [0, 0] : lqr.computeInput(xd, measured);

  const f = pol.derivative(sim.x, sim.u);
  if (!f) {
    // The firmware treats a singular mass matrix as a hard fault and cuts the
    // motors; do the same rather than integrating garbage.
    sim.fallen = true;
    sim.fallenAt = sim.time;
    return;
  }
  for (let i = 0; i < 4; i++) sim.x[i] += f[i] * dt;

  // --- world pose --------------------------------------------------------
  sim.pose.psi += sim.x[3] * dt;
  sim.pose.x += sim.x[2] * Math.cos(sim.pose.psi) * dt;
  sim.pose.y += sim.x[2] * Math.sin(sim.pose.psi) * dt;
  sim.time += dt;

  if (!sim.fallen && Math.abs(sim.x[0]) > 1.05) {
    sim.fallen = true;
    sim.fallenAt = sim.time;
    sim.x[2] = 0;
    sim.x[3] = 0;
  }
  if (sim.fallen && sim.time - sim.fallenAt > 1.6) resetSim();
}

/** Momentary shove, so the balance controller has something to reject. */
function push(strength) {
  if (sim.fallen) return;
  sim.x[1] += strength;
}

// --------------------------------------------------------------------------
// log replay
// --------------------------------------------------------------------------
const player = new LogPlayer();
const replay = { active: false, playing: true, rate: 1, pose: { x: 0, y: 0, psi: 0 } };

async function loadLog(file) {
  logStatus.textContent = 'loading…';
  try {
    await player.load(file);
    replay.pose = { x: 0, y: 0, psi: 0 };
    logStatus.textContent = `${player.rows.length} samples · ${player.duration.toFixed(1)} s`;
    scrub.max = String(player.duration);
  } catch (err) {
    logStatus.textContent = String(err.message || err);
  }
}

function stepReplay(dt) {
  if (replay.playing) player.advance(dt * replay.rate);
  const s = player.sample();
  if (!s) return null;
  replay.pose.psi += s.psiDot * dt * replay.rate;
  replay.pose.x += s.v * Math.cos(replay.pose.psi) * dt * replay.rate;
  replay.pose.y += s.v * Math.sin(replay.pose.psi) * dt * replay.rate;
  return s;
}

// --------------------------------------------------------------------------
// UI wiring
// --------------------------------------------------------------------------
const el = (id) => document.getElementById(id);
const banner = el('banner');
const readout = el('readout');
const logStatus = el('log-status');
const scrub = el('scrub');
const hud = new Hud(el('traces'));

const modeButtons = [...document.querySelectorAll('[data-mode]')];
function setMode(mode) {
  replay.active = mode === 'replay';
  for (const b of modeButtons) b.classList.toggle('on', b.dataset.mode === mode);
  el('replay-controls').hidden = !replay.active;
  el('drive-controls').hidden = replay.active;
  document.body.classList.toggle('replay', replay.active);
  hud.clear();
  if (replay.active && !player.rows.length) loadLog(el('log-select').value);
  else if (!replay.active) resetSim();
}
for (const b of modeButtons) b.addEventListener('click', () => setMode(b.dataset.mode));

const logSelect = el('log-select');
for (const l of LOGS) {
  const opt = document.createElement('option');
  opt.value = l.file;
  opt.textContent = l.label;
  logSelect.append(opt);
}
logSelect.addEventListener('change', () => loadLog(logSelect.value));

el('play-pause').addEventListener('click', (e) => {
  replay.playing = !replay.playing;
  e.currentTarget.textContent = replay.playing ? 'Pause' : 'Play';
});
el('rate').addEventListener('input', (e) => {
  replay.rate = Number(e.target.value);
  el('rate-value').textContent = `${replay.rate.toFixed(2)}×`;
});
scrub.addEventListener('input', (e) => {
  player.seek(Number(e.target.value));
});

const heightSlider = el('height');
heightSlider.addEventListener('input', (e) => {
  sim.hTarget = Number(e.target.value);
});
el('roll').addEventListener('input', (e) => {
  sim.phiCmd = Number(e.target.value);
  el('roll-value').textContent = `${sim.phiCmd.toFixed(0)}°`;
});
el('follow').addEventListener('change', (e) => {
  follow = e.target.checked;
});
el('show-com').addEventListener('change', (e) => {
  robot.showCom = e.target.checked;
});
el('use-meshes').addEventListener('change', (e) => {
  robot.useMeshes = e.target.checked;
});
el('use-estimate').addEventListener('change', (e) => {
  sim.useEstimate = e.target.checked;
});
el('noise').addEventListener('input', (e) => {
  sim.noiseScale = Number(e.target.value);
  el('noise-value').textContent = `${sim.noiseScale.toFixed(2)}×`;
});
el('reset').addEventListener('click', resetSim);
el('push').addEventListener('click', () => push(3.2));

let follow = true;

// --------------------------------------------------------------------------
// keyboard, shared with the on-screen key pads
// --------------------------------------------------------------------------
const KEY_ALIASES = {
  ' ': 'space',
  arrowup: 'q',
  arrowdown: 'e',
  arrowleft: 'a',
  arrowright: 'd',
};
const WATCHED = new Set(['w', 'a', 's', 'd', 'q', 'e', 'space', 'p', 'r']);

function normalise(e) {
  const k = e.key.toLowerCase();
  return KEY_ALIASES[k] || k;
}

function keyDown(key) {
  if (!WATCHED.has(key)) return;
  if (keys.has(key)) return;
  keys.add(key);
  document.getElementById(`key-${key}`)?.classList.add('active');

  if (key === 'p') push(3.2);
  if (key === 'r') resetSim();
  if (key === 'q' || key === 'e') {
    const d = key === 'q' ? 0.015 : -0.015;
    sim.hTarget = clamp(sim.hTarget + d, LIMITS.HEIGHT_MIN, LIMITS.HEIGHT_MAX);
    heightSlider.value = String(sim.hTarget);
  }
}

function keyUp(key) {
  keys.delete(key);
  document.getElementById(`key-${key}`)?.classList.remove('active');
}

document.addEventListener('keydown', (e) => {
  if (e.target.matches('input, select, textarea')) return;
  const k = normalise(e);
  if (WATCHED.has(k)) e.preventDefault();
  keyDown(k);
});
document.addEventListener('keyup', (e) => keyUp(normalise(e)));
window.addEventListener('blur', () => [...keys].forEach(keyUp));

// A panel control keeps focus after a click, which quietly kills the keyboard:
// the keydown handler below stands aside for focused form controls (so typing
// is never hijacked), so after ticking one checkbox W/A/S/D and Space stop
// driving the robot and Space just re-toggles that checkbox instead. Dropping
// focus on pointer release fixes it for mouse and touch, while leaving a
// keyboard user who tabbed to the control with its normal behaviour.
for (const control of document.querySelectorAll('#panel button, #panel input, #panel select')) {
  control.addEventListener('pointerup', () => control.blur());
}

// Pointer support for the on-screen pads.
for (const pad of document.querySelectorAll('[data-key]')) {
  const key = pad.dataset.key;
  pad.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    pad.setPointerCapture(e.pointerId);
    keyDown(key);
  });
  const release = () => keyUp(key);
  pad.addEventListener('pointerup', release);
  pad.addEventListener('pointercancel', release);
  pad.addEventListener('lostpointercapture', release);
}

// --------------------------------------------------------------------------
// main loop
// --------------------------------------------------------------------------
let last = performance.now();
let accumulator = 0;
let hudClock = 0;

function frame(now) {
  requestAnimationFrame(frame);
  const wall = Math.min((now - last) / 1000, 0.1); // ignore long tab stalls
  last = now;

  let sample;
  if (replay.active) {
    sample = stepReplay(wall);
    if (sample) {
      pol.setHR(sample.h, 0);
      pol.computeComAndInertia();
      robot.update(
        { ...replay.pose, theta: sample.theta, phi: 0 },
        pol,
        wall,
        { v: sample.v, psiDot: sample.psiDot },
      );
      scrub.value = String(player.t);
    }
  } else {
    // Fixed 8 ms steps, matching the firmware's sampling time.
    accumulator += wall;
    let steps = 0;
    while (accumulator >= DT && steps < 12) {
      stepSim(DT);
      accumulator -= DT;
      steps++;
    }
    robot.update(
      { ...sim.pose, theta: sim.x[0], phi: pol.phiApplied },
      pol,
      wall,
      { v: sim.x[2], psiDot: sim.x[3] },
    );
  }

  // Keep the shadow frustum over the robot wherever it has driven to.
  const ground = replay.active ? replay.pose : sim.pose;
  followLight(keyLight, [ground.x, ground.y]);

  // Camera. The chase camera drives camera.position directly, so OrbitControls
  // has to stand down while it is on - left enabled, it would recompute the
  // position from its own spherical offset every update and undo the lerp.
  const target = robot.group.getWorldPosition(new THREE.Vector3());
  target.y += 0.14;
  if (follow) {
    controls.enabled = false;
    const psi = (replay.active ? replay.pose.psi : sim.pose.psi) + CHASE.azimuth;
    const want = new THREE.Vector3(
      target.x - Math.cos(psi) * CHASE.distance,
      target.y + CHASE.rise,
      target.z + Math.sin(psi) * CHASE.distance,
    );
    camera.position.lerp(want, 1 - Math.exp(-3.5 * wall));
    camera.lookAt(target);
    controls.target.copy(target); // keep it in sync for the handover
  } else {
    controls.enabled = true;
    controls.target.lerp(target, 1 - Math.exp(-8 * wall));
    controls.update();
  }

  // telemetry, at a calm 25 Hz
  hudClock += wall;
  if (hudClock > 0.04) {
    hudClock = 0;
    if (replay.active && sample) {
      hud.push({
        theta: sample.theta,
        v: sample.v,
        psiDot: sample.psiDot,
        tauR: sample.tauR,
        tauL: sample.tauL,
      });
      updateReadout(sample.h, sample.theta, sample.v, sample.psiDot, [sample.tauR, sample.tauL], sample.t);
    } else if (!replay.active) {
      hud.push({
        theta: sim.x[0],
        thetaHat: sim.xHat[0],
        v: sim.x[2],
        psiDot: sim.x[3],
        tauR: sim.u[0],
        tauL: sim.u[1],
      });
      updateReadout(sim.hCmd, sim.x[0], sim.x[2], sim.x[3], sim.u, sim.time);
    }
    hud.draw();
    banner.classList.toggle('show', !replay.active && sim.fallen);
  }

  renderer.render(scene, camera);
}

const deg = (r) => ((r * 180) / Math.PI).toFixed(1);

function updateReadout(h, theta, v, psiDot, u, t) {
  const hips = pol.thetaHips.map((a) => (a * 180) / Math.PI);
  // HEIGHT_MAX sits exactly on the servo stop, so allow a degree of slack and
  // only flag commands the hardware would really have to clip.
  const TOL = 1;
  const cmdR = HIP_SERVO.right.center - hips[0];
  const cmdL = HIP_SERVO.left.center - hips[1];
  const clipped =
    cmdR < HIP_SERVO.right.min - TOL || cmdR > HIP_SERVO.right.max + TOL ||
    cmdL < HIP_SERVO.left.min - TOL || cmdL > HIP_SERVO.left.max + TOL;

  readout.innerHTML = `
    <div><span>t</span><b>${t.toFixed(2)} s</b></div>
    <div><span>height h</span><b>${(h * 1000).toFixed(0)} mm</b></div>
    <div><span>pitch θ</span><b>${deg(theta)}°</b></div>
    <div><span>θ equilibrium</span><b>${deg(pol.thetaEquilibrium())}°</b></div>
    <div><span>speed v</span><b>${v.toFixed(3)} m/s</b></div>
    <div><span>yaw rate ψ̇</span><b>${psiDot.toFixed(3)} rad/s</b></div>
    <div><span>τ right / left</span><b>${u[0].toFixed(3)} / ${u[1].toFixed(3)} N·m</b></div>
    <div><span>hip servos</span><b class="${clipped ? 'warn' : ''}">${hips[0].toFixed(1)}° / ${hips[1].toFixed(1)}°</b></div>
    <div><span>CoM offset</span><b>${(pol.p_bcom[0] * 1000).toFixed(1)}, ${(pol.p_bcom[2] * 1000).toFixed(1)} mm</b></div>
    ${replay.active ? '' : `
    <div><span>θ estimate error</span><b>${deg(sim.estError[0])}°</b></div>
    <div><span>v estimate error</span><b>${sim.estError[2].toFixed(4)} m/s</b></div>
    <div><span>tr(P)</span><b>${ekf.uncertainty().toExponential(2)}</b></div>`}
  `;
}

// CAD meshes load in the background: the simulator is fully usable on the
// primitives, and each link swaps over as its mesh arrives.
loadLinkMeshes().then(({ meshes, missing }) => {
  const found = Object.keys(meshes);
  const toggle = el('use-meshes');
  const status = el('mesh-status');

  if (!found.length) {
    toggle.disabled = true;
    status.textContent = 'no CAD meshes converted yet — showing primitives';
    return;
  }

  robot.applyMeshes(meshes);
  robot.useMeshes = toggle.checked;
  status.textContent = missing.length
    ? `${found.length} of ${Object.keys(LINK_MESHES).length} links from CAD; the rest are primitives`
    : `all ${found.length} links from CAD`;
});

resize();
resetSim();
setMode('simulate');
requestAnimationFrame(frame);
