/**
 * Three.js rendering of the POW robot.
 *
 * The scene graph mirrors the way the model is written down:
 *
 *   base      at the wheel-axle midpoint, on the ground plane + R, yawed by psi
 *    |- axle  rolled by phi; carries the two spinning wheels
 *    \- roll  rolled by phi
 *        \- pitch   pitched by theta, about the axle
 *            \- body at (0, 0, h): the body frame POL works in, so the joint
 *                     positions coming out of POL.legPoints drop straight in
 *
 * Everything inside is expressed in the robot convention (+x forward, +y left,
 * +z up); the caller's `root` group carries the single rotation that maps it
 * onto three.js's y-up world.
 */

import * as THREE from 'three';
import { GEOM } from './params.js';

const COLORS = {
  chassis: 0x2b3440,
  chassisEdge: 0x8ea3bd,
  thighActive: 0xe6a53c,
  thighPassive: 0x7f8b9c,
  calf: 0xc9d3e0,
  tyre: 0x1b1f26,
  rim: 0xe6a53c,
  com: 0xff4d6d,
};

/** Chassis box dimensions recovered from the body inertia tensor in Params.h. */
const CHASSIS = { sx: 0.207, sy: 0.144, sz: 0.114, cx: 0.0137, cy: 0, cz: 0.0349 };

/** Places a capsule between two points given in the parent's frame. */
function segment(material, radius) {
  const geo = new THREE.CylinderGeometry(radius, radius, 1, 12);
  geo.translate(0, 0.5, 0); // grow along +y from the origin
  const mesh = new THREE.Mesh(geo, material);
  mesh.castShadow = true;
  return mesh;
}

function setSegment(mesh, from, to) {
  const a = new THREE.Vector3(...from);
  const b = new THREE.Vector3(...to);
  const d = b.clone().sub(a);
  const len = d.length();
  mesh.position.copy(a);
  mesh.scale.set(1, Math.max(len, 1e-6), 1);
  if (len > 1e-9) {
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
  }
}

export class Robot3D {
  constructor() {
    this.group = new THREE.Group(); // the "base" node

    const mat = (color, opts = {}) =>
      new THREE.MeshStandardMaterial({ color, metalness: 0.25, roughness: 0.55, ...opts });

    // --- wheels: on the axle, independent of body pitch -------------------
    this.axle = new THREE.Group();
    this.group.add(this.axle);

    this.wheels = [];
    for (const side of [0, 1]) {
      const sign = side === 0 ? -1 : 1;
      const wheel = new THREE.Group();
      wheel.position.set(0, sign * GEOM.L, 0);

      const tyreGeo = new THREE.CylinderGeometry(GEOM.R, GEOM.R, 0.026, 32);
      const tyre = new THREE.Mesh(tyreGeo, mat(COLORS.tyre, { roughness: 0.9, metalness: 0.05 }));
      tyre.castShadow = true;
      wheel.add(tyre);

      const rimGeo = new THREE.CylinderGeometry(GEOM.R * 0.42, GEOM.R * 0.42, 0.03, 24);
      wheel.add(new THREE.Mesh(rimGeo, mat(COLORS.rim, { metalness: 0.6, roughness: 0.35 })));

      // A spoke, so the wheel's rotation is legible at a glance.
      const spokeGeo = new THREE.BoxGeometry(GEOM.R * 1.7, 0.032, 0.012);
      const spoke = new THREE.Mesh(spokeGeo, mat(COLORS.rim, { metalness: 0.6 }));
      wheel.add(spoke);

      this.axle.add(wheel);
      this.wheels.push(wheel);
    }

    // --- body and legs -----------------------------------------------------
    this.roll = new THREE.Group();
    this.group.add(this.roll);
    this.pitch = new THREE.Group();
    this.roll.add(this.pitch);
    this.body = new THREE.Group();
    this.pitch.add(this.body);

    const chassisGeo = new THREE.BoxGeometry(CHASSIS.sx, CHASSIS.sy, CHASSIS.sz);
    const chassis = new THREE.Mesh(chassisGeo, mat(COLORS.chassis, { roughness: 0.65 }));
    chassis.position.set(CHASSIS.cx, CHASSIS.cy, CHASSIS.cz);
    chassis.castShadow = true;
    this.body.add(chassis);
    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(chassisGeo),
      new THREE.LineBasicMaterial({ color: COLORS.chassisEdge }),
    );
    edges.position.copy(chassis.position);
    this.body.add(edges);

    // A short nose so "forward" is unambiguous while driving.
    const nose = new THREE.Mesh(
      new THREE.ConeGeometry(0.022, 0.05, 4),
      mat(0xe6a53c, { metalness: 0.5 }),
    );
    nose.rotation.z = -Math.PI / 2;
    nose.position.set(CHASSIS.cx + CHASSIS.sx / 2 + 0.022, 0, CHASSIS.cz);
    this.body.add(nose);

    const matActive = mat(COLORS.thighActive, { metalness: 0.4 });
    const matPassive = mat(COLORS.thighPassive, { metalness: 0.5 });
    const matCalf = mat(COLORS.calf, { metalness: 0.35, roughness: 0.45 });

    this.legs = [0, 1].map(() => {
      const thighActive = segment(matActive, 0.011); // B -> C
      const thighPassive = segment(matPassive, 0.011); // A -> D
      const calfMain = segment(matCalf, 0.013); // D -> E (wheel axle)
      const calfCoupler = segment(matCalf, 0.009); // D -> C
      const hip = new THREE.Mesh(
        new THREE.CylinderGeometry(0.017, 0.017, 0.03, 16),
        mat(0x39424f, { metalness: 0.5 }),
      );
      hip.rotation.x = Math.PI / 2; // servo axis along body y
      const knee = new THREE.Mesh(
        new THREE.SphereGeometry(0.013, 14, 10),
        mat(0x39424f, { metalness: 0.5 }),
      );
      const parts = { thighActive, thighPassive, calfMain, calfCoupler, hip, knee };
      for (const p of Object.values(parts)) this.body.add(p);
      return parts;
    });

    // Centre of mass marker, useful when watching the robot squat.
    this.comMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.014, 16, 12),
      new THREE.MeshBasicMaterial({ color: COLORS.com, transparent: true, opacity: 0.85 }),
    );
    this.body.add(this.comMarker);

    this.wheelAngle = [0, 0];
  }

  set showCom(v) {
    this.comMarker.visible = v;
  }

  /**
   * Push a simulation state onto the scene graph.
   *
   * pose  {x, y, psi, theta, phi}  robot pose in the world (m, rad)
   * pol   the POL instance, already stepped through computeComAndInertia()
   * dt    seconds since the last call, used to roll the wheels
   * vel   {v, psiDot} used to roll the wheels
   */
  update(pose, pol, dt, vel) {
    this.group.position.set(pose.x, pose.y, GEOM.R);
    this.group.rotation.set(0, 0, pose.psi);

    this.axle.rotation.set(pose.phi, 0, 0);
    this.roll.rotation.set(pose.phi, 0, 0);
    this.pitch.rotation.set(0, pose.theta, 0);

    // The body frame origin sits h above the axle midpoint.
    this.body.position.set(0, 0, pol.h);

    for (const side of [0, 1]) {
      const sign = side === 0 ? -1 : 1;
      const p = pol.legPoints(side);
      const leg = this.legs[side];
      setSegment(leg.thighActive, p.B, p.C);
      setSegment(leg.thighPassive, p.A, p.D);
      setSegment(leg.calfMain, p.D, p.E);
      setSegment(leg.calfCoupler, p.D, p.C);
      leg.hip.position.set(...p.B);
      leg.knee.position.set(...p.D);

      // Ground speed of this wheel: v +/- L * psi_dot.
      const vWheel = vel.v + sign * -1 * GEOM.L * vel.psiDot;
      this.wheelAngle[side] += (vWheel / GEOM.R) * dt;
      this.wheels[side].rotation.set(0, this.wheelAngle[side], 0);
    }

    this.comMarker.position.set(...pol.p_bcom);
  }
}

/**
 * Ground plane, grid and lighting.
 * `scene` carries the background and fog; `parent` is the +z up group that
 * everything else is authored in.
 */
export function buildScene(scene, parent) {
  scene.background = new THREE.Color(0x0e1116);
  scene.fog = new THREE.Fog(0x0e1116, 6, 24);

  const hemi = new THREE.HemisphereLight(0x9fc4ff, 0x2a3038, 1.15);
  parent.add(hemi);

  const key = new THREE.DirectionalLight(0xffffff, 1.6);
  key.position.set(3, -4, 6);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  const d = 2.2;
  key.shadow.camera.left = -d;
  key.shadow.camera.right = d;
  key.shadow.camera.top = d;
  key.shadow.camera.bottom = -d;
  key.shadow.camera.far = 20;
  key.shadow.bias = -0.0012;
  parent.add(key);
  const fill = new THREE.DirectionalLight(0x4b6ea8, 0.35);
  fill.position.set(-4, 3, 2);
  parent.add(fill);

  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(80, 80),
    new THREE.MeshStandardMaterial({ color: 0x232833, roughness: 0.95, metalness: 0 }),
  );
  ground.receiveShadow = true;
  parent.add(ground);

  // GridHelper is authored in the xz plane; stand it up into our xy ground.
  const grid = new THREE.GridHelper(80, 160, 0x4a5c78, 0x2f3644);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = 0.001;
  parent.add(grid);

  return { key };
}
