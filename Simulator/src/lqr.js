/**
 * Gain-scheduled LQR wheel controller.
 *
 * Port of ESP32/WBR_Control/VYBController.h. The gain table holds one 2x4
 * matrix per 10 mm of commanded body height, starting at HEIGHT_MIN; the
 * controller interpolates linearly between the two bracketing entries and
 * saturates each wheel torque at MAX_TORQUE.
 */

import { LQR_GAINS, LIMITS } from './params.js';
import { clamp } from './mat.js';

const STEP = 0.01; // one gain per 10 mm, as in computeGainK

export class LQRController {
  constructor() {
    this.K = LQR_GAINS[0].slice();
    this.u = [0, 0];
    this.saturated = false;
  }

  /** Select and interpolate the gain for a commanded height h (m). */
  computeGain(h) {
    const t = (h - LIMITS.HEIGHT_MIN) / STEP;
    const idx = Math.floor(t);

    if (idx < 0) {
      this.K = LQR_GAINS[0].slice();
    } else if (idx >= LQR_GAINS.length - 1) {
      this.K = LQR_GAINS[LQR_GAINS.length - 1].slice();
    } else {
      const r = t - idx;
      const lo = LQR_GAINS[idx];
      const hi = LQR_GAINS[idx + 1];
      this.K = lo.map((v, i) => v * (1 - r) + hi[i] * r);
    }
    return this.K;
  }

  /** u = sat(K (x_d - x)). Returns [tau_RW, tau_LW] in N*m. */
  computeInput(xd, x) {
    const e = [xd[0] - x[0], xd[1] - x[1], xd[2] - x[2], xd[3] - x[3]];
    const raw = [
      this.K[0] * e[0] + this.K[1] * e[1] + this.K[2] * e[2] + this.K[3] * e[3],
      this.K[4] * e[0] + this.K[5] * e[1] + this.K[6] * e[2] + this.K[7] * e[3],
    ];
    const lim = LIMITS.MAX_TORQUE;
    this.saturated = Math.abs(raw[0]) > lim || Math.abs(raw[1]) > lim;
    this.u = [clamp(raw[0], -lim, lim), clamp(raw[1], -lim, lim)];
    return this.u;
  }
}
