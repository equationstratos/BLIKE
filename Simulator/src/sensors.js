/**
 * Synthetic IMU and wheel-encoder measurements.
 *
 * The firmware's measurement vector z is
 *   z[0..2]  accelerometer, body frame            (m/s^2)
 *   z[3..5]  rate gyroscope, body frame           (rad/s)
 *   z[6]     right wheel motor speed, body-relative (rad/s)
 *   z[7]     left wheel motor speed                (rad/s)
 *
 * We generate it from the true state with the same observation model the EKF
 * inverts (EKF.h :: predict_measurement) - but evaluated with the *true*
 * accelerations. The firmware's h_obs sets theta_ddot, v_dot and psi_ddot to
 * zero (the terms are commented out in EKF.h), so feeding it real ones here
 * reproduces the modelling error the filter actually faces on the robot,
 * instead of a filter marking its own homework.
 */

import { GRAVITY } from './params.js';

/**
 * Noise standard deviations, from the variances the firmware carries in the
 * commented-out "identified" R matrix in EKF.h. The active R it runs with is
 * a much coarser hand-tuning; this is the one that looks measured.
 */
export const SENSOR_VARIANCE = [
  1.54239e-1, 1.93287e-1, 2.63191e-1, // accelerometer
  3.11351e-3, 4.12642e-3, 5.37135e-3, // gyroscope
  1e-5, 1e-5, // wheel encoders
];

/** Box-Muller, so the noise is actually Gaussian rather than uniform-ish. */
function gaussian() {
  let u = 0;
  while (u === 0) u = Math.random();
  const v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

/**
 * @param x     true state [theta, theta_dot, v, psi_dot]
 * @param xdot  true derivative, for the acceleration terms
 * @param pol   POL instance (for h, L, R)
 * @param noise scale on the standard deviations; 0 gives perfect sensors
 */
export function measure(x, xdot, pol, noise = 1) {
  const [theta, thetaDot, v, psiDot] = x;
  const thetaDdot = xdot ? xdot[1] : 0;
  const vDot = xdot ? xdot[2] : 0;
  const psiDdot = xdot ? xdot[3] : 0;

  const z = observationModel(theta, thetaDot, v, psiDot, pol, thetaDdot, vDot, psiDdot);
  if (noise > 0) {
    for (let i = 0; i < 8; i++) {
      z[i] += gaussian() * Math.sqrt(SENSOR_VARIANCE[i]) * noise;
    }
  }
  return z;
}

/**
 * The observation model h(x). With the acceleration arguments left at zero
 * this is exactly EKF.h :: predict_measurement, which is how the filter uses it.
 */
export function observationModel(
  theta, thetaDot, v, psiDot, pol,
  thetaDdot = 0, vDot = 0, psiDdot = 0,
) {
  const h = pol.h;
  const L = pol.L;
  const R = pol.R;
  const g = GRAVITY;

  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  const ct2 = ct * ct;
  const sc = st * ct;
  const pd2 = psiDot * psiDot;

  return [
    h * thetaDdot + vDot * ct - g * st - h * pd2 * sc,
    psiDot * v + h * psiDdot * st + h * psiDot * thetaDot * ct * 2,
    -h * thetaDot * thetaDot + g * ct + vDot * st - pd2 * (h - h * ct2),
    -psiDot * st,
    thetaDot,
    psiDot * ct,
    thetaDot - v / R - (L * psiDot) / R,
    -thetaDot + v / R - (L * psiDot) / R,
  ];
}

/** Jacobian dh/dx of the observation model, as an 8x4 row-major array. */
export function observationJacobian(theta, thetaDot, v, psiDot, pol) {
  const h = pol.h;
  const L = pol.L;
  const R = pol.R;
  const g = GRAVITY;

  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  const ct2 = ct * ct;
  const st2 = st * st;
  const sc = st * ct;
  const pd2 = psiDot * psiDot;

  // The firmware evaluates this with theta_ddot = v_dot = psi_ddot = 0, to
  // match its h_obs; we do the same so filter and model stay consistent.
  const H = new Float64Array(32);
  const put = (row, col, val) => {
    H[row * 4 + col] = val;
  };

  put(0, 0, pd2 * (h * st2 - h * ct2) - g * ct);
  put(1, 0, -h * psiDot * thetaDot * st * 2);
  put(2, 0, -g * st - h * pd2 * sc * 2);
  put(3, 0, -psiDot * ct);
  put(5, 0, -psiDot * st);

  put(1, 1, h * psiDot * ct * 2);
  put(2, 1, h * thetaDot * -2);
  put(4, 1, 1);
  put(6, 1, 1);
  put(7, 1, -1);

  put(1, 2, psiDot);
  put(6, 2, -1 / R);
  put(7, 2, 1 / R);

  put(0, 3, h * psiDot * sc * -2);
  put(1, 3, v + h * thetaDot * ct * 2);
  put(2, 3, psiDot * (h - h * ct2) * -2);
  put(3, 3, -st);
  put(5, 3, ct);
  put(6, 3, -L / R);
  put(7, 3, -L / R);

  return H;
}
