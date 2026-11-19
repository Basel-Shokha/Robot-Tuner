// draw.js - canvas helpers: a pan/zoom world view (metres, +y up), and drawing of walls, robots, rays, paths.
export function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || '#888'; }
export const PALETTE = ['#1f6feb', '#e8590c', '#2f9e44', '#ae3ec9', '#f08c00', '#0c8599', '#e03131', '#5c7cfa', '#66a80f', '#d6336c'];

export class View {
  /** canvas: element; opts: {scale px per metre, cx, cy centre (m), height px or aspect} */
  constructor(canvas, opts = {}) {
    this.c = canvas; this.g = canvas.getContext('2d');
    this.scale = opts.scale || 400; this.cx = opts.cx || 0; this.cy = opts.cy || 0;
    this.aspect = opts.aspect || 0.5; this.minH = opts.minH || 240; this.maxH = opts.maxH || 640;
    this.interactive = opts.interactive !== false;
    this.onChange = null;
    this.resize();
    new ResizeObserver(() => { this.resize(); this.onChange && this.onChange(); }).observe(canvas.parentElement);
    if (this.interactive) this._bind();
  }
  resize() {
    const w = this.c.parentElement.clientWidth || 600;
    const h = Math.max(this.minH, Math.min(this.maxH, Math.round(w * this.aspect)));
    const d = window.devicePixelRatio || 1;
    this.c.width = Math.round(w * d); this.c.height = Math.round(h * d);
    this.c.style.height = h + 'px';
    this.w = w; this.h = h; this.dpr = d;
  }
  // world (m) -> css px
  sx(x) { return this.w / 2 + (x - this.cx) * this.scale; }
  sy(y) { return this.h / 2 - (y - this.cy) * this.scale; }
  wx(px) { return (px - this.w / 2) / this.scale + this.cx; }
  wy(py) { return -(py - this.h / 2) / this.scale + this.cy; }
  begin() {
    const g = this.g;
    g.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    g.fillStyle = css('--surface'); g.fillRect(0, 0, this.w, this.h);
    return g;
  }
  grid(step = 0.1) {
    const g = this.g;
    let s = step; while (s * this.scale < 18) s *= 5;
    g.strokeStyle = css('--grid'); g.lineWidth = 1; g.beginPath();
    for (let x = Math.floor(this.wx(0) / s) * s; x < this.wx(this.w); x += s) { g.moveTo(this.sx(x), 0); g.lineTo(this.sx(x), this.h); }
    for (let y = Math.floor(this.wy(this.h) / s) * s; y < this.wy(0); y += s) { g.moveTo(0, this.sy(y)); g.lineTo(this.w, this.sy(y)); }
    g.stroke();
    return s;
  }
  scaleBar() {
    const g = this.g; let m = 0.01;
    for (const c of [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5]) if (c * this.scale < 120) m = c;
    const x = this.w - 20 - m * this.scale, y = this.h - 16;
    g.strokeStyle = css('--muted'); g.lineWidth = 2; g.beginPath(); g.moveTo(x, y); g.lineTo(x + m * this.scale, y); g.stroke();
    g.fillStyle = css('--muted'); g.font = '11px system-ui'; g.textAlign = 'center';
    g.fillText(m >= 1 ? m + ' m' : Math.round(m * 1000) + ' mm', x + m * this.scale / 2, y - 5);
  }
  walls(flat, color = css('--wall'), width = 2.5) {
    const g = this.g; g.strokeStyle = color; g.lineWidth = width; g.lineCap = 'round'; g.beginPath();
    for (let i = 0; i < flat.length; i += 4) { g.moveTo(this.sx(flat[i]), this.sy(flat[i + 1])); g.lineTo(this.sx(flat[i + 2]), this.sy(flat[i + 3])); }
    g.stroke();
  }
  polyline(pts, color, width = 2, dash = null, alpha = 1) {
    if (pts.length < 2) return;
    const g = this.g; g.save(); g.globalAlpha = alpha; g.strokeStyle = color; g.lineWidth = width; if (dash) g.setLineDash(dash);
    g.beginPath(); g.moveTo(this.sx(pts[0][0]), this.sy(pts[0][1]));
    for (let i = 1; i < pts.length; i++) g.lineTo(this.sx(pts[i][0]), this.sy(pts[i][1]));
    g.stroke(); g.restore();
  }
  poly(pts, fill, stroke, alpha = 1) {
    const g = this.g; g.save(); g.globalAlpha = alpha; g.beginPath();
    pts.forEach((p, i) => (i ? g.lineTo(this.sx(p[0]), this.sy(p[1])) : g.moveTo(this.sx(p[0]), this.sy(p[1]))));
    g.closePath(); if (fill) { g.fillStyle = fill; g.fill(); } if (stroke) { g.strokeStyle = stroke; g.lineWidth = 1.5; g.stroke(); }
    g.restore();
  }
  /** footprint: [body, wheel, wheel, ...] polygons from Sim.footprint() */
  robot(foot, color = css('--robot'), alpha = 1, heading = null) {
    foot.slice(1).forEach((w) => this.poly(w, '#333a', null, alpha));
    this.poly(foot[0], color + '40', color, alpha);
    if (heading !== null) {
      const b = foot[0]; const fx = (b[0][0] + b[1][0]) / 2, fy = (b[0][1] + b[1][1]) / 2;
      const cx = (b[0][0] + b[2][0]) / 2, cy = (b[0][1] + b[2][1]) / 2;
      this.polyline([[cx, cy], [fx, fy]], color, 2, null, alpha);
    }
  }
  /** rays from Sim.rays(): cone outline + reading */
  rays(rays, color = css('--ray'), alpha = 0.9) {
    const g = this.g; g.save(); g.globalAlpha = alpha;
    for (const r of rays) {
      const d = Math.min(r.d, r.max);
      g.fillStyle = color + '22'; g.beginPath(); g.moveTo(this.sx(r.x), this.sy(r.y));
      for (const a of [r.a - r.half, r.a + r.half]) g.lineTo(this.sx(r.x + Math.cos(a) * d), this.sy(r.y + Math.sin(a) * d));
      g.closePath(); g.fill();
      g.strokeStyle = color; g.lineWidth = 1.5; g.beginPath(); g.moveTo(this.sx(r.x), this.sy(r.y));
      g.lineTo(this.sx(r.x + Math.cos(r.a) * d), this.sy(r.y + Math.sin(r.a) * d)); g.stroke();
      g.fillStyle = color; g.beginPath(); g.arc(this.sx(r.x + Math.cos(r.a) * d), this.sy(r.y + Math.sin(r.a) * d), 2.5, 0, 7); g.fill();
    }
    g.restore();
  }
  dot(x, y, r, color) { const g = this.g; g.fillStyle = color; g.beginPath(); g.arc(this.sx(x), this.sy(y), r, 0, 7); g.fill(); }
  text(x, y, s, color = css('--muted'), align = 'left') { const g = this.g; g.fillStyle = color; g.font = '12px system-ui'; g.textAlign = align; g.fillText(s, this.sx(x), this.sy(y)); }
  _bind() {
    let drag = null;
    this.c.addEventListener('pointerdown', (e) => { if (e.button !== 0 || this.onPointerDown && this.onPointerDown(e)) return; drag = { x: e.clientX, y: e.clientY, cx: this.cx, cy: this.cy }; this.c.setPointerCapture(e.pointerId); });
    this.c.addEventListener('pointermove', (e) => {
      if (this.onPointerMove && this.onPointerMove(e)) return;
      if (!drag) return;
      this.cx = drag.cx - (e.clientX - drag.x) / this.scale; this.cy = drag.cy + (e.clientY - drag.y) / this.scale;
      this.userMoved = true; this.onChange && this.onChange();
    });
    this.c.addEventListener('pointerup', (e) => { drag = null; this.onPointerUp && this.onPointerUp(e); });
    this.c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const r = this.c.getBoundingClientRect(); const px = e.clientX - r.left, py = e.clientY - r.top;
      const wx = this.wx(px), wy = this.wy(py);
      this.scale = Math.max(20, Math.min(20000, this.scale * Math.exp(-e.deltaY * 0.0015)));
      this.cx = wx - (px - this.w / 2) / this.scale; this.cy = wy + (py - this.h / 2) / this.scale;
      this.userMoved = true; this.onChange && this.onChange();
    }, { passive: false });
  }
  /** event -> world coordinates */
  eventWorld(e) { const r = this.c.getBoundingClientRect(); return [this.wx(e.clientX - r.left), this.wy(e.clientY - r.top)]; }
  fit(minx, miny, maxx, maxy, pad = 0.1) {
    const w = Math.max(1e-3, maxx - minx), h = Math.max(1e-3, maxy - miny);
    this.scale = Math.min(this.w / (w * (1 + 2 * pad)), this.h / (h * (1 + 2 * pad)));
    this.cx = (minx + maxx) / 2; this.cy = (miny + maxy) / 2;
  }
}

// Robot drawn from its JSON (builder, previews): shapes in the robot frame, millimetres.
export function robotShapes(r) {
  const b = r.body, w = r.wheels;
  const body = [[b.front_mm, -b.width_mm / 2], [b.front_mm, b.width_mm / 2], [-b.rear_mm, b.width_mm / 2], [-b.rear_mm, -b.width_mm / 2]];
  const wheels = [];
  const xs = r.drive === '4wd' ? [w.x_mm + w.wheelbase_mm / 2, w.x_mm - w.wheelbase_mm / 2] : [w.x_mm];
  for (const x of xs) for (const side of [1, -1]) {
    const cy = side * w.track_mm / 2, hl = w.diameter_mm / 2, hw = w.width_mm / 2;
    wheels.push([[x + hl, cy - hw], [x + hl, cy + hw], [x - hl, cy + hw], [x - hl, cy - hw]]);
  }
  return { body, wheels, caster: r.caster.enabled ? { x: r.caster.x_mm, d: r.caster.diameter_mm } : null, sensors: r.sensors };
}
