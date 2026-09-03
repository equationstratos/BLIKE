/**
 * Small dense matrix helpers, for the pieces of the estimator that are not
 * 3x3. Matrices are row-major: { r, c, d } with d.length === r * c.
 *
 * Nothing here is optimised; the biggest inverse the simulator ever takes is
 * the 8x8 innovation covariance, once per 8 ms control step.
 */

export function mat(r, c, values) {
  const m = { r, c, d: new Float64Array(r * c) };
  if (values) m.d.set(values);
  return m;
}

export function eye(n, scale = 1) {
  const m = mat(n, n);
  for (let i = 0; i < n; i++) m.d[i * n + i] = scale;
  return m;
}

export function diag(values) {
  const n = values.length;
  const m = mat(n, n);
  for (let i = 0; i < n; i++) m.d[i * n + i] = values[i];
  return m;
}

export function fromColumn(values) {
  return mat(values.length, 1, values);
}

export function clone(A) {
  return { r: A.r, c: A.c, d: Float64Array.from(A.d) };
}

export const get = (A, i, j) => A.d[i * A.c + j];
export const set = (A, i, j, v) => {
  A.d[i * A.c + j] = v;
};

export function mul(A, B) {
  if (A.c !== B.r) throw new Error(`shape mismatch: ${A.r}x${A.c} * ${B.r}x${B.c}`);
  const C = mat(A.r, B.c);
  for (let i = 0; i < A.r; i++) {
    for (let k = 0; k < A.c; k++) {
      const a = A.d[i * A.c + k];
      if (a === 0) continue;
      for (let j = 0; j < B.c; j++) C.d[i * C.c + j] += a * B.d[k * B.c + j];
    }
  }
  return C;
}

export function transpose(A) {
  const T = mat(A.c, A.r);
  for (let i = 0; i < A.r; i++) {
    for (let j = 0; j < A.c; j++) T.d[j * T.c + i] = A.d[i * A.c + j];
  }
  return T;
}

export function add(A, B) {
  const C = clone(A);
  for (let i = 0; i < C.d.length; i++) C.d[i] += B.d[i];
  return C;
}

export function sub(A, B) {
  const C = clone(A);
  for (let i = 0; i < C.d.length; i++) C.d[i] -= B.d[i];
  return C;
}

export function scale(A, s) {
  const C = clone(A);
  for (let i = 0; i < C.d.length; i++) C.d[i] *= s;
  return C;
}

export function isFinite_(A) {
  for (let i = 0; i < A.d.length; i++) if (!Number.isFinite(A.d[i])) return false;
  return true;
}

/**
 * Gauss-Jordan inverse with partial pivoting.
 * Returns null when the matrix is singular or turns non-finite, so callers can
 * fault out the way the firmware does rather than propagate NaN.
 */
export function inverse(A) {
  const n = A.r;
  if (n !== A.c) throw new Error('inverse of a non-square matrix');
  const a = Float64Array.from(A.d);
  const inv = eye(n).d;

  for (let col = 0; col < n; col++) {
    // pivot
    let best = col;
    let bestAbs = Math.abs(a[col * n + col]);
    for (let row = col + 1; row < n; row++) {
      const v = Math.abs(a[row * n + col]);
      if (v > bestAbs) {
        bestAbs = v;
        best = row;
      }
    }
    if (!(bestAbs > 1e-14)) return null;

    if (best !== col) {
      for (let j = 0; j < n; j++) {
        let t = a[col * n + j];
        a[col * n + j] = a[best * n + j];
        a[best * n + j] = t;
        t = inv[col * n + j];
        inv[col * n + j] = inv[best * n + j];
        inv[best * n + j] = t;
      }
    }

    const piv = a[col * n + col];
    for (let j = 0; j < n; j++) {
      a[col * n + j] /= piv;
      inv[col * n + j] /= piv;
    }

    for (let row = 0; row < n; row++) {
      if (row === col) continue;
      const f = a[row * n + col];
      if (f === 0) continue;
      for (let j = 0; j < n; j++) {
        a[row * n + j] -= f * a[col * n + j];
        inv[row * n + j] -= f * inv[col * n + j];
      }
    }
  }

  const out = mat(n, n, inv);
  return isFinite_(out) ? out : null;
}
