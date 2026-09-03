/** Minimal 3x3 / 3-vector helpers. Matrices are row-major flat arrays of 9. */

export const mat3 = {
  zero: () => new Float64Array(9),
  identity: () => Float64Array.from([1, 0, 0, 0, 1, 0, 0, 0, 1]),

  /** Rotation of `t` radians about the y axis (POL.h :: rotation_matrix_y). */
  rotY(t) {
    const c = Math.cos(t);
    const s = Math.sin(t);
    return Float64Array.from([c, 0, s, 0, 1, 0, -s, 0, c]);
  },

  mul(A, B) {
    const C = new Float64Array(9);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        let s = 0;
        for (let k = 0; k < 3; k++) s += A[i * 3 + k] * B[k * 3 + j];
        C[i * 3 + j] = s;
      }
    }
    return C;
  },

  mulT(A, B) {
    // A * B^T
    const C = new Float64Array(9);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        let s = 0;
        for (let k = 0; k < 3; k++) s += A[i * 3 + k] * B[j * 3 + k];
        C[i * 3 + j] = s;
      }
    }
    return C;
  },

  apply(A, v) {
    return [
      A[0] * v[0] + A[1] * v[1] + A[2] * v[2],
      A[3] * v[0] + A[4] * v[1] + A[5] * v[2],
      A[6] * v[0] + A[7] * v[1] + A[8] * v[2],
    ];
  },

  /** In-place C += s * A. */
  addScaled(C, A, s) {
    for (let i = 0; i < 9; i++) C[i] += s * A[i];
    return C;
  },

  det(A) {
    return (
      A[0] * (A[4] * A[8] - A[5] * A[7]) -
      A[1] * (A[3] * A[8] - A[5] * A[6]) +
      A[2] * (A[3] * A[7] - A[4] * A[6])
    );
  },

  /** Inverse, or null when the matrix is singular / non-finite. */
  inverse(A) {
    const d = mat3.det(A);
    if (!Number.isFinite(d) || Math.abs(d) < 1e-18) return null;
    const id = 1 / d;
    const M = new Float64Array(9);
    M[0] = (A[4] * A[8] - A[5] * A[7]) * id;
    M[1] = (A[2] * A[7] - A[1] * A[8]) * id;
    M[2] = (A[1] * A[5] - A[2] * A[4]) * id;
    M[3] = (A[5] * A[6] - A[3] * A[8]) * id;
    M[4] = (A[0] * A[8] - A[2] * A[6]) * id;
    M[5] = (A[2] * A[3] - A[0] * A[5]) * id;
    M[6] = (A[3] * A[7] - A[4] * A[6]) * id;
    M[7] = (A[1] * A[6] - A[0] * A[7]) * id;
    M[8] = (A[0] * A[4] - A[1] * A[3]) * id;
    for (let i = 0; i < 9; i++) if (!Number.isFinite(M[i])) return null;
    return M;
  },
};

export const vec3 = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  len: (a) => Math.hypot(a[0], a[1], a[2]),
};

/** m * (|r|^2 * I - r r^T), the parallel-axis term of a rigid body. */
export function parallelAxis(r, m) {
  const nn = vec3.dot(r, r);
  const M = new Float64Array(9);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      M[i * 3 + j] = m * ((i === j ? nn : 0) - r[i] * r[j]);
    }
  }
  return M;
}

export const clamp = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);
