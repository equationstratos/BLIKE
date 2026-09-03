/** Rolling strip charts for the telemetry panel. */

const TRACES = [
  { key: 'theta', label: 'pitch θ', unit: '°', color: '#e6a53c', range: 12 },
  { key: 'v', label: 'speed v', unit: 'm/s', color: '#5ec8a0', range: 1.2 },
  { key: 'psiDot', label: 'yaw rate ψ̇', unit: 'rad/s', color: '#6ba8ff', range: 1.2 },
  { key: 'tau', label: 'wheel torque', unit: 'N·m', color: '#ff6b8a', range: 0.9 },
];

const SPAN = 320; // samples kept per trace

export class Hud {
  constructor(container) {
    this.traces = TRACES.map((t) => {
      const wrap = document.createElement('div');
      wrap.className = 'trace';

      const head = document.createElement('div');
      head.className = 'trace-head';
      const name = document.createElement('span');
      name.textContent = t.label;
      const value = document.createElement('b');
      value.style.color = t.color;
      head.append(name, value);

      const canvas = document.createElement('canvas');
      canvas.className = 'trace-canvas';
      wrap.append(head, canvas);
      container.append(wrap);

      return { ...t, canvas, ctx: canvas.getContext('2d'), value, data: [], data2: [] };
    });
    this.resize();
  }

  resize() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    for (const t of this.traces) {
      const r = t.canvas.getBoundingClientRect();
      t.canvas.width = Math.max(1, Math.round(r.width * dpr));
      t.canvas.height = Math.max(1, Math.round(r.height * dpr));
      t.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      t.w = r.width;
      t.h = r.height;
    }
  }

  /** Drop the history, so a mode switch starts from a blank chart. */
  clear() {
    for (const t of this.traces) {
      t.data.length = 0;
      t.data2.length = 0;
    }
  }

  /** sample: {theta (rad), v, psiDot, tauR, tauL} */
  push(sample) {
    for (const t of this.traces) {
      let a;
      let b = null;
      if (t.key === 'theta') {
        a = (sample.theta * 180) / Math.PI;
        // ghost line: the EKF's estimate, when there is one
        if (sample.thetaHat !== undefined) b = (sample.thetaHat * 180) / Math.PI;
      } else if (t.key === 'tau') {
        a = sample.tauR;
        b = sample.tauL;
      } else a = sample[t.key];

      t.data.push(a);
      if (t.data.length > SPAN) t.data.shift();
      if (b !== null) {
        t.data2.push(b);
        if (t.data2.length > SPAN) t.data2.shift();
      }
      t.value.textContent = `${a >= 0 ? '+' : ''}${a.toFixed(2)} ${t.unit}`;
    }
  }

  draw() {
    for (const t of this.traces) {
      const { ctx, w, h } = t;
      if (!w) continue;
      ctx.clearRect(0, 0, w, h);

      // zero line
      ctx.strokeStyle = 'rgba(255,255,255,0.13)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(0, h / 2);
      ctx.lineTo(w, h / 2);
      ctx.stroke();

      // autoscale, but never below the nominal range so small signals stay small
      let peak = t.range;
      for (const series of [t.data, t.data2]) {
        for (const v of series) peak = Math.max(peak, Math.abs(v));
      }

      const plot = (series, color, width) => {
        if (series.length < 2) return;
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.beginPath();
        series.forEach((v, i) => {
          const x = (i / (SPAN - 1)) * w;
          const y = h / 2 - (v / peak) * (h / 2 - 2);
          i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
        });
        ctx.stroke();
      };

      plot(t.data2, 'rgba(255,255,255,0.35)', 1);
      plot(t.data, t.color, 1.5);
    }
  }
}
