/**
 * Playback of flight logs recorded on the real robot.
 *
 * The CSVs under data/ come from SeungbinOh/Pow_WBR_Project (MATLAB/log_plot
 * and MATLAB/test_data), thinned to about 4000 samples each. Columns are the
 * ones the firmware's Logger writes: estimated states (`*_hat`), references
 * (`*_d`) and the wheel torques actually commanded.
 */

export const LOGS = [
  { file: 'balancing.csv', label: 'Balancing (mid height)' },
  { file: 'squat.csv', label: 'Squat / height sweep' },
  { file: 'max-velocity.csv', label: 'Maximum velocity run' },
  { file: 'max-yaw-rate.csv', label: 'Maximum yaw rate' },
  { file: 'yawing.csv', label: 'Yawing' },
];

function parseCsv(text) {
  const lines = text.trim().split(/\r?\n/);
  const header = lines[0].split(',').map((s) => s.trim());
  const rows = new Array(lines.length - 1);
  for (let i = 1; i < lines.length; i++) {
    const parts = lines[i].split(',');
    const row = {};
    for (let j = 0; j < header.length; j++) {
      if (!header[j]) continue;
      row[header[j]] = parseFloat(parts[j]);
    }
    rows[i - 1] = row;
  }
  return rows;
}

export class LogPlayer {
  constructor() {
    this.rows = [];
    this.t = 0; // seconds into the log
    this.duration = 0;
    this.name = '';
  }

  async load(file) {
    const res = await fetch(new URL(`../data/${file}`, import.meta.url));
    if (!res.ok) throw new Error(`cannot read ${file}: HTTP ${res.status}`);
    this.rows = parseCsv(await res.text());
    // TimeStamp is in milliseconds since the run started.
    const t0 = this.rows[0].TimeStamp;
    for (const r of this.rows) r._t = (r.TimeStamp - t0) / 1000;
    this.duration = this.rows[this.rows.length - 1]._t;
    this.t = 0;
    this.name = file;
    return this;
  }

  seek(t) {
    this.t = Math.max(0, Math.min(this.duration, t));
  }

  advance(dt) {
    this.t += dt;
    if (this.t > this.duration) this.t -= this.duration; // loop
  }

  /** Linearly interpolated sample at the current time. */
  sample() {
    const rows = this.rows;
    if (!rows.length) return null;

    // rows are time-ordered, so a bisect is both cheap and scrub-friendly
    let lo = 0;
    let hi = rows.length - 1;
    while (lo < hi - 1) {
      const mid = (lo + hi) >> 1;
      if (rows[mid]._t <= this.t) lo = mid;
      else hi = mid;
    }
    const a = rows[lo];
    const b = rows[hi];
    const span = b._t - a._t;
    const r = span > 1e-9 ? (this.t - a._t) / span : 0;
    const mix = (k, fallback = 0) => {
      const va = a[k];
      const vb = b[k];
      if (!Number.isFinite(va)) return fallback;
      return Number.isFinite(vb) ? va + (vb - va) * r : va;
    };

    return {
      t: this.t,
      h: mix('h_d', 0.2),
      theta: mix('theta_hat'),
      thetaDot: mix('theta_dot_hat'),
      v: mix('v_hat'),
      psiDot: mix('psi_dot_hat'),
      thetaRef: mix('theta_d'),
      vRef: mix('v_d'),
      psiDotRef: mix('psi_dot_d'),
      tauR: mix('tau_RW'),
      tauL: mix('tau_LW'),
    };
  }
}
