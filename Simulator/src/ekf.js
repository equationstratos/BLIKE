/**
 * Extended Kalman filter for the reduced 4-state model.
 *
 * Port of ESP32/WBR_Control/EKF.h. State x = [theta, theta_dot, v, psi_dot];
 * measurement z is the 8-vector of accelerometer, gyroscope and wheel speeds
 * described in sensors.js.
 *
 * One deviation from the firmware, marked below: it propagates the covariance
 * with the *continuous* Jacobian df/dx while integrating the state with an
 * explicit Euler step. Those are inconsistent - the covariance of an Euler step
 * propagates through the Jacobian of that step, I + (df/dx) dt - and using the
 * continuous one makes P grow without bound. We use the discrete Jacobian.
 */

import {
  mat, eye, diag, fromColumn, mul, transpose, add, sub, inverse, get,
} from './linalg.js';
import { DT } from './params.js';
import { observationModel, observationJacobian } from './sensors.js';

/** Measurement noise covariance, as tuned in the firmware's active R_cov. */
export const R_COV = [1e-1, 4, 1e-1, 1, 4.12642e-6, 1, 0, 0];
/** Process noise covariance, the firmware's active Q_cov. */
export const Q_COV = [0, 1, 1, 1];

export class EKF {
  constructor(pol) {
    this.pol = pol;
    this.R = diag(R_COV);
    this.Q = diag(Q_COV);
    this.reset();
  }

  reset(x = [0, 0, 0, 0]) {
    this.x = x.slice();
    this.P = eye(4, 1);
    this.diverged = false;
    this.innovation = [0, 0, 0, 0, 0, 0, 0, 0];
  }

  /**
   * One predict/update cycle. `u` is the torque applied over the step, `z` the
   * measurement. Returns the updated estimate, or null if the model faulted -
   * which is how the firmware detects divergence and cuts the motors.
   */
  step(z, u, dt = DT) {
    const pol = this.pol;

    // --- predict ---------------------------------------------------------
    const f = pol.derivative(this.x, u);
    const fx = pol.stateJacobian(this.x, u);
    if (!f || !fx) {
      this.diverged = true;
      return null;
    }

    const xPred = this.x.map((v, i) => v + f[i] * dt);

    // Discrete-time Jacobian of the Euler step, not the continuous df/dx the
    // firmware hands to this line.
    const Phi = eye(4);
    for (let i = 0; i < 16; i++) Phi.d[i] += fx[i] * dt;

    const Ppred = add(mul(mul(Phi, this.P), transpose(Phi)), this.Q);

    // --- update ----------------------------------------------------------
    const hObs = observationModel(xPred[0], xPred[1], xPred[2], xPred[3], pol);
    const H = mat(8, 4, observationJacobian(xPred[0], xPred[1], xPred[2], xPred[3], pol));
    const Ht = transpose(H);

    const S = add(mul(mul(H, Ppred), Ht), this.R);
    const Sinv = inverse(S);
    if (!Sinv) {
      this.diverged = true;
      return null;
    }

    const K = mul(mul(Ppred, Ht), Sinv);
    const innov = fromColumn(z.map((zi, i) => zi - hObs[i]));
    const correction = mul(K, innov);

    const xNew = xPred.map((v, i) => v + correction.d[i]);
    if (!xNew.every(Number.isFinite)) {
      this.diverged = true;
      return null;
    }

    this.P = mul(sub(eye(4), mul(K, H)), Ppred);
    this.x = xNew;
    this.innovation = innov.d;
    return this.x;
  }

  /** Trace of P, a single-number summary of how confident the filter is. */
  uncertainty() {
    return get(this.P, 0, 0) + get(this.P, 1, 1) + get(this.P, 2, 2) + get(this.P, 3, 3);
  }
}
