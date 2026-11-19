// plot.js - small, fast time-series plots on canvas (no library, works offline).
import { css } from './draw.js';

export class TimePlot {
  /** parent: element; opts {title, series:[{name, color}], window: seconds, min, max (fixed y) , height} */
  constructor(parent, opts) {
    this.o = Object.assign({ window: 10, height: 140 }, opts);
    this.box = document.createElement('div'); this.box.className = 'plot';
    const h = document.createElement('h4'); h.textContent = opts.title || '';
    this.legend = document.createElement('div'); this.legend.className = 'legend';
    this.c = document.createElement('canvas');
    this.box.append(h, this.c, this.legend);
    parent.append(this.box);
    this.setSeries(opts.series || []);
    this.resize();
    new ResizeObserver(() => { this.resize(); this.draw(); }).observe(this.box);
  }
  setSeries(series) {
    this.series = series.map((s) => ({ ...s, t: [], y: [] }));
    this.legend.innerHTML = this.series.map((s) => `<span><i style="background:${s.color}"></i>${s.name}</span>`).join('');
  }
  setTitle(s) { this.box.querySelector('h4').textContent = s; }
  resize() {
    const w = this.box.clientWidth - 16 || 300, d = window.devicePixelRatio || 1;
    this.c.width = w * d; this.c.height = this.o.height * d; this.c.style.width = w + 'px'; this.c.style.height = this.o.height + 'px';
    this.w = w; this.h = this.o.height; this.d = d;
  }
  clear() { for (const s of this.series) { s.t.length = 0; s.y.length = 0; } }
  push(t, values) {
    values.forEach((v, i) => {
      const s = this.series[i]; if (!s) return;
      s.t.push(t); s.y.push(v);
      if (!this.o.keepAll) while (s.t.length > 2 && s.t[0] < t - this.o.window) { s.t.shift(); s.y.shift(); }
    });
  }
  /** whole-series data (sweep mode): arrays of {t, y} */
  setData(i, ts, ys) { const s = this.series[i]; s.t = ts.slice(); s.y = ys.slice(); }
  draw() {
    const g = this.c.getContext('2d'); g.setTransform(this.d, 0, 0, this.d, 0, 0);
    g.clearRect(0, 0, this.w, this.h);
    let t0 = Infinity, t1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (const s of this.series) for (let k = 0; k < s.t.length; k++) {
      if (s.t[k] < t0) t0 = s.t[k]; if (s.t[k] > t1) t1 = s.t[k];
      const y = s.y[k]; if (Number.isFinite(y)) { if (y < y0) y0 = y; if (y > y1) y1 = y; }
    }
    if (!Number.isFinite(t0)) { t0 = 0; t1 = this.o.window; }
    if (!this.o.keepAll) t0 = Math.max(t0, t1 - this.o.window);
    if (t1 - t0 < 1e-6) t1 = t0 + 1;
    if (this.o.min !== undefined) y0 = Math.min(this.o.min, y0); if (this.o.max !== undefined) y1 = Math.max(this.o.max, y1);
    if (this.o.fixed) { y0 = this.o.min; y1 = this.o.max; }
    if (!Number.isFinite(y0)) { y0 = -1; y1 = 1; }
    if (y1 - y0 < 1e-9) { y0 -= 1; y1 += 1; }
    const pad = (y1 - y0) * 0.08; y0 -= pad; y1 += pad;
    const L = 40, R = 6, T = 6, B = 16, W = this.w - L - R, H = this.h - T - B;
    const X = (t) => L + (t - t0) / (t1 - t0) * W, Y = (y) => T + (1 - (y - y0) / (y1 - y0)) * H;
    g.strokeStyle = css('--grid'); g.fillStyle = css('--muted'); g.font = '10px system-ui'; g.lineWidth = 1;
    const ticks = niceTicks(y0, y1, 4);
    g.textAlign = 'right';
    for (const v of ticks) { g.beginPath(); g.moveTo(L, Y(v)); g.lineTo(L + W, Y(v)); g.stroke(); g.fillText(fmtTick(v), L - 4, Y(v) + 3); }
    if (y0 < 0 && y1 > 0) { g.strokeStyle = css('--muted'); g.beginPath(); g.moveTo(L, Y(0)); g.lineTo(L + W, Y(0)); g.stroke(); }
    g.textAlign = 'center';
    for (const v of niceTicks(t0, t1, 5)) g.fillText(fmtTick(v) + (this.o.tUnit ?? ' s'), X(v), this.h - 3);
    for (const s of this.series) {
      if (s.t.length < 2) continue;
      g.strokeStyle = s.color; g.lineWidth = s.width || 1.6; g.setLineDash(s.dash || []); g.beginPath();
      let first = true;
      for (let k = 0; k < s.t.length; k++) {
        if (s.t[k] < t0) continue;
        const x = X(s.t[k]), y = Y(Math.max(y0, Math.min(y1, s.y[k])));
        if (first) { g.moveTo(x, y); first = false; } else g.lineTo(x, y);
      }
      g.stroke(); g.setLineDash([]);
    }
  }
}
function niceTicks(a, b, n) {
  const span = b - a, step0 = span / n, mag = Math.pow(10, Math.floor(Math.log10(step0)));
  const step = [1, 2, 5, 10].map((m) => m * mag).find((s) => span / s <= n) || 10 * mag;
  const out = []; for (let v = Math.ceil(a / step) * step; v <= b + 1e-12; v += step) out.push(+v.toFixed(10));
  return out;
}
function fmtTick(v) { const a = Math.abs(v); return a >= 100 ? v.toFixed(0) : a >= 10 ? v.toFixed(1) : a >= 1 ? v.toFixed(2).replace(/0$/, '') : a === 0 ? '0' : v.toFixed(3).replace(/0+$/, ''); }
