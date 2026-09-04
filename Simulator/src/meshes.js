/**
 * Loader for the POWM link meshes produced by tools/stl2powm.py.
 *
 * Each file holds one link's geometry in that link's own frame — the same
 * frames POL.h works in — so a loaded mesh drops straight onto the matching
 * joint group with no extra transform. See tools/stl2powm.py for the format.
 *
 * Meshes are optional: any link without a file keeps the primitive stand-in, so
 * the simulator runs with none, some, or all of them present.
 */

import * as THREE from 'three';

const MAGIC = 0x4d574f50; // 'POWM' little-endian

/**
 * Link name -> mesh file, following the naming in the project's model.sdf.
 * `null` marks a link whose mesh has not been converted yet.
 */
export const LINK_MESHES = {
  body: 'MainBody_Visual',
  thighActiveRight: 'ThighLink_Active_Right_Visual',
  thighActiveLeft: 'ThighLink_Active_Left_Visual',
  thighPassiveRight: 'ThighLink_Passive_Right_Visual',
  thighPassiveLeft: 'ThighLink_Passive_Left_Visual',
  calfRight: 'Calf_Link_Right_Visual',
  calfLeft: 'Calf_Link_Left_Visual',
  wheelRight: 'Wheel_Right_Visual',
  wheelLeft: 'Wheel_Left_Visual',
};

/** Parse one POWM buffer into a BufferGeometry. */
export function parsePowm(buffer) {
  const view = new DataView(buffer);
  if (view.getUint32(0, true) !== MAGIC) throw new Error('not a POWM file');

  const version = view.getUint16(4, true);
  if (version !== 1) throw new Error(`unsupported POWM version ${version}`);

  const vertCount = view.getUint32(8, true);
  const triCount = view.getUint32(12, true);

  const lo = [view.getFloat32(16, true), view.getFloat32(20, true), view.getFloat32(24, true)];
  const hi = [view.getFloat32(28, true), view.getFloat32(32, true), view.getFloat32(36, true)];

  const quant = new Int16Array(buffer, 40, vertCount * 3);
  // The writer pads the position block to a 4-byte boundary: an odd vertex
  // count would otherwise leave the index block at a 2-byte offset, where a
  // Uint32Array view cannot start.
  const indexOffset = 40 + vertCount * 6 + ((vertCount * 6) % 4 ? 2 : 0);
  const indices = new Uint32Array(buffer, indexOffset, triCount * 3);

  // Undo the bounding-box quantisation.
  const positions = new Float32Array(vertCount * 3);
  for (let axis = 0; axis < 3; axis++) {
    const span = hi[axis] - lo[axis];
    const scale = (span < 1e-9 ? 0 : span) / 65535;
    for (let i = 0; i < vertCount; i++) {
      positions[i * 3 + axis] = lo[axis] + (quant[i * 3 + axis] + 32768) * scale;
    }
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geo.setIndex(new THREE.BufferAttribute(indices, 1));

  return creasedNormals(geo, CREASE_ANGLE);
}

/** Faces meeting at a shallower angle than this get smoothed together. */
const CREASE_ANGLE = (42 * Math.PI) / 180;

/**
 * Build normals with a crease angle: neighbouring faces are averaged together
 * only where they meet gently, so machined edges stay crisp while a panel made
 * of many decimated triangles reads as one flat surface.
 *
 * Plain flat shading makes decimation error visible — a vertex moved a few
 * tenths of a millimetre tilts a large triangle enough to catch the light
 * differently from the one beside it, and a flat panel comes out looking
 * shattered. Fully smooth shading has the opposite problem, rounding off every
 * machined corner. This is the usual middle course, and it is what CAD viewers
 * do.
 */
function creasedNormals(geo, threshold) {
  const position = geo.getAttribute('position');
  const index = geo.getIndex();
  const triCount = index.count / 3;
  const pos = position.array;
  const idx = index.array;

  // Face normals, unnormalised so their length weights by twice the area.
  const faceNormal = new Float32Array(triCount * 3);
  const unit = new Float32Array(triCount * 3);
  for (let f = 0; f < triCount; f++) {
    const a = idx[f * 3] * 3;
    const b = idx[f * 3 + 1] * 3;
    const c = idx[f * 3 + 2] * 3;
    const abx = pos[b] - pos[a];
    const aby = pos[b + 1] - pos[a + 1];
    const abz = pos[b + 2] - pos[a + 2];
    const acx = pos[c] - pos[a];
    const acy = pos[c + 1] - pos[a + 1];
    const acz = pos[c + 2] - pos[a + 2];
    const nx = aby * acz - abz * acy;
    const ny = abz * acx - abx * acz;
    const nz = abx * acy - aby * acx;
    faceNormal[f * 3] = nx;
    faceNormal[f * 3 + 1] = ny;
    faceNormal[f * 3 + 2] = nz;
    const len = Math.hypot(nx, ny, nz) || 1;
    unit[f * 3] = nx / len;
    unit[f * 3 + 1] = ny / len;
    unit[f * 3 + 2] = nz / len;
  }

  // Faces touching each vertex, as a flat CSR-style adjacency.
  const vertCount = position.count;
  const counts = new Uint32Array(vertCount + 1);
  for (let i = 0; i < idx.length; i++) counts[idx[i] + 1]++;
  for (let v = 0; v < vertCount; v++) counts[v + 1] += counts[v];
  const cursor = counts.slice();
  const adjacency = new Uint32Array(idx.length);
  for (let f = 0; f < triCount; f++) {
    for (let k = 0; k < 3; k++) adjacency[cursor[idx[f * 3 + k]]++] = f;
  }

  const cosLimit = Math.cos(threshold);
  const outPos = new Float32Array(triCount * 9);
  const outNrm = new Float32Array(triCount * 9);

  for (let f = 0; f < triCount; f++) {
    const fx = unit[f * 3];
    const fy = unit[f * 3 + 1];
    const fz = unit[f * 3 + 2];
    for (let k = 0; k < 3; k++) {
      const v = idx[f * 3 + k];
      let nx = 0;
      let ny = 0;
      let nz = 0;
      for (let a = counts[v]; a < counts[v + 1]; a++) {
        const g = adjacency[a];
        const dot = fx * unit[g * 3] + fy * unit[g * 3 + 1] + fz * unit[g * 3 + 2];
        if (dot >= cosLimit) {
          nx += faceNormal[g * 3];
          ny += faceNormal[g * 3 + 1];
          nz += faceNormal[g * 3 + 2];
        }
      }
      const len = Math.hypot(nx, ny, nz);
      const o = f * 9 + k * 3;
      if (len > 1e-20) {
        outNrm[o] = nx / len;
        outNrm[o + 1] = ny / len;
        outNrm[o + 2] = nz / len;
      } else {
        outNrm[o] = fx;
        outNrm[o + 1] = fy;
        outNrm[o + 2] = fz;
      }
      const p = v * 3;
      outPos[o] = pos[p];
      outPos[o + 1] = pos[p + 1];
      outPos[o + 2] = pos[p + 2];
    }
  }

  geo.dispose();
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', new THREE.BufferAttribute(outPos, 3));
  out.setAttribute('normal', new THREE.BufferAttribute(outNrm, 3));
  out.computeBoundingSphere();
  return out;
}

/**
 * Fetch every link mesh that exists, in parallel.
 * Missing files are not an error — they simply leave that link on its
 * primitive. Returns { meshes: {link: BufferGeometry}, missing: [link] }.
 */
export async function loadLinkMeshes(baseUrl = new URL('../meshes/', import.meta.url)) {
  const meshes = {};
  const missing = [];

  await Promise.all(
    Object.entries(LINK_MESHES).map(async ([link, file]) => {
      if (!file) {
        missing.push(link);
        return;
      }
      try {
        const res = await fetch(new URL(`${file}.powm`, baseUrl));
        if (!res.ok) {
          // Not converted yet — expected, and the link keeps its primitive.
          missing.push(link);
          return;
        }
        meshes[link] = parsePowm(await res.arrayBuffer());
      } catch (err) {
        // A mesh that is present but unreadable is a real problem: say so
        // rather than quietly falling back and looking like it was absent.
        console.error(`link mesh ${file}.powm failed to load:`, err);
        missing.push(link);
      }
    }),
  );

  return { meshes, missing };
}
