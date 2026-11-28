// robot3d.js - a simple 3D model of the robot (box body, wheels, caster ball, sensors with their view direction), built
// from the robot's numbers and redrawn when they change. Turn it with the mouse / a finger (OrbitControls).
// show(path): "MEASURE BY HAND": turns the view to a good side and draws a measuring line from where to start to where
// to end, with the value. World frame = the robot frame: x forward, y left, z up, origin = middle of the drive axle (mm).
import * as T from '../vendor/three/three.bundle.js';
import { t } from './common.js';

const PALETTE = ['#2f6fdf', '#e8a10c', '#0f6e52', '#c8352e', '#7b4fd6'];
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim() || '#888';
const mm = (x) => (+x || 0) / 1000;   // scene units: metres
/** body height for the picture only (not a robot value): in proportion to the robot's size */
const heightOf = (r) => Math.max(25, Math.min(90, 0.3 * Math.max(r.body.width_mm, r.body.front_mm + r.body.rear_mm), 1.2 * r.wheels.diameter_mm));

/** what to measure for each value: a and b in mm (robot frame), the view to look from */
function measureOf(path, r) {
  const b = r.body, w = r.wheels, c = r.caster, H = heightOf(r), rad = w.diameter_mm / 2;
  const bottom = rad * 0.45, top = bottom + H, yOut = b.width_mm / 2 + 25, xw = +w.x_mm || 0;
  const m = /^sensors\.(\d+)\.(\w+)$/.exec(path);
  if (m) {
    const s = r.sensors[+m[1]]; if (!s) return null;
    if (m[2] === 'x_mm') return { a: [0, s.y_mm, top + 6], b: [s.x_mm, s.y_mm, top + 6], view: 'top', axle: true, sensor: +m[1] };
    if (m[2] === 'y_mm') return { a: [s.x_mm, 0, top + 6], b: [s.x_mm, s.y_mm, top + 6], view: 'top', center: true, sensor: +m[1] };
    if (m[2] === 'angle_deg') return { arc: { at: [s.x_mm, s.y_mm, top + 6], deg: s.angle_deg }, view: 'top', center: true, sensor: +m[1] };
    return null;
  }
  switch (path) {
    case 'body.front_mm': return { a: [0, -yOut, top], b: [b.front_mm, -yOut, top], view: 'top', axle: true, part: 'body' };
    case 'body.rear_mm': return { a: [0, -yOut, top], b: [-b.rear_mm, -yOut, top], view: 'top', axle: true, part: 'body' };
    case 'body.width_mm': return { a: [b.front_mm + 25, -b.width_mm / 2, top], b: [b.front_mm + 25, b.width_mm / 2, top], view: 'top', part: 'body' };
    case 'wheels.diameter_mm': return { a: [xw, -(w.track_mm / 2 + w.width_mm / 2 + 12), 0], b: [xw, -(w.track_mm / 2 + w.width_mm / 2 + 12), 2 * rad], view: 'side', part: 'wheel' };
    case 'wheels.width_mm': return { a: [xw, -(w.track_mm / 2 - w.width_mm / 2), 2 * rad + 8], b: [xw, -(w.track_mm / 2 + w.width_mm / 2), 2 * rad + 8], view: 'front', part: 'wheel' };
    case 'wheels.track_mm': return { a: [xw, -w.track_mm / 2, rad], b: [xw, w.track_mm / 2, rad], view: 'front', part: 'wheel' };
    case 'wheels.track_effective_mm': return { a: [xw, -w.track_mm / 2, rad], b: [xw, w.track_mm / 2, rad], view: 'front', part: 'wheel', note: true };
    case 'wheels.x_mm': return { a: [0, -yOut, 2], b: [xw, -yOut, 2], view: 'top', axle: true };
    case 'wheels.wheelbase_mm': return { a: [xw + w.wheelbase_mm / 2, -(w.track_mm / 2 + w.width_mm), rad], b: [xw - w.wheelbase_mm / 2, -(w.track_mm / 2 + w.width_mm), rad], view: 'side', part: 'wheel' };
    case 'caster.x_mm': return { a: [0, c.y_mm || 0, 4], b: [c.x_mm, c.y_mm || 0, 4], view: 'top', axle: true, part: 'caster' };
    case 'caster.y_mm': return { a: [c.x_mm, 0, 4], b: [c.x_mm, c.y_mm || 0, 4], view: 'top', center: true, part: 'caster' };
    case 'caster.diameter_mm': return { a: [c.x_mm, (c.y_mm || 0) - c.diameter_mm / 2 - 10, 0], b: [c.x_mm, (c.y_mm || 0) - c.diameter_mm / 2 - 10, c.diameter_mm], view: 'side', part: 'caster' };
    default: return null;
  }
}
/** the values that have a "MEASURE BY HAND" button */
export const HAND = new Set(['body.front_mm', 'body.rear_mm', 'body.width_mm', 'wheels.diameter_mm', 'wheels.width_mm', 'wheels.track_mm',
  'wheels.x_mm', 'wheels.wheelbase_mm', 'caster.x_mm', 'caster.y_mm', 'caster.diameter_mm']);
export const isHand = (path) => HAND.has(path) || /^sensors\.\d+\.(x_mm|y_mm|angle_deg)$/.test(path);

function label(text, color) {
  const c = document.createElement('canvas'), g = c.getContext('2d'), fs = 44;
  g.font = `700 ${fs}px system-ui, sans-serif`;
  const w = Math.ceil(g.measureText(text).width) + 28;
  c.width = w; c.height = fs + 22;
  g.font = `700 ${fs}px system-ui, sans-serif`;
  g.fillStyle = 'rgba(255,255,255,.95)'; g.strokeStyle = color; g.lineWidth = 5;
  g.beginPath(); g.roundRect(3, 3, w - 6, fs + 16, 12); g.fill(); g.stroke();
  g.fillStyle = '#13212e'; g.textBaseline = 'middle'; g.fillText(text, 14, (fs + 22) / 2 + 1);
  const sp = new T.Sprite(new T.SpriteMaterial({ map: new T.CanvasTexture(c), depthTest: false }));
  const h = 0.018; sp.scale.set(h * w / (fs + 22), h, 1); sp.renderOrder = 10;
  return sp;
}
function lineOf(pts, color, dashed = false) {
  const g = new T.BufferGeometry(); g.setAttribute('position', new T.Float32BufferAttribute(pts.flatMap((p) => p.map(mm)), 3));
  const mat = dashed ? new T.LineDashedMaterial({ color, dashSize: 0.006, gapSize: 0.004, depthTest: false }) : new T.LineBasicMaterial({ color, depthTest: false, linewidth: 2 });
  const l = new T.Line(g, mat); if (dashed) l.computeLineDistances(); l.renderOrder = 9; return l;
}

/** a red laser dot (the start and the end of a measurement) */
function laserDot(p) {
  const g = new T.Group();
  const core = new T.Mesh(new T.SphereGeometry(mm(2.2), 12, 8), new T.MeshBasicMaterial({ color: 0xff1a1a, depthTest: false }));
  const glow = new T.Mesh(new T.SphereGeometry(mm(5), 12, 8), new T.MeshBasicMaterial({ color: 0xff1a1a, transparent: true, opacity: 0.25, depthTest: false }));
  core.renderOrder = glow.renderOrder = 11; g.add(glow, core); g.position.set(...p.map(mm)); return g;
}
export class Robot3D {
  constructor(box) {
    this.box = box;
    this.renderer = new T.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    box.append(this.renderer.domElement);
    this.scene = new T.Scene();
    this.camera = new T.PerspectiveCamera(40, 1, 0.005, 20);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(-0.32, -0.38, 0.3);
    this.controls = new T.OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true; this.controls.minDistance = 0.08; this.controls.maxDistance = 2.5;
    this.scene.add(new T.HemisphereLight(0xffffff, 0x8899aa, 1.6));
    const sun = new T.DirectionalLight(0xffffff, 1.6); sun.position.set(-0.5, -0.8, 1.2); this.scene.add(sun);
    const grid = new T.GridHelper(1.2, 24, 0x9aa6b0, 0xd3dae0); grid.rotation.x = Math.PI / 2; this.scene.add(grid);
    this.robotG = new T.Group(); this.measureG = new T.Group(); this.scene.add(this.robotG, this.measureG);
    this.anim = null;
    new ResizeObserver(() => this.resize()).observe(box);
    const loop = () => { requestAnimationFrame(loop); this.tick(); };
    loop();
  }
  resize() {
    const w = this.box.clientWidth, h = this.box.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false); this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
  }
  tick() {
    if (this.anim) {
      const k = Math.min(1, (performance.now() - this.anim.t0) / 450), e = k * k * (3 - 2 * k);
      this.camera.position.lerpVectors(this.anim.p0, this.anim.p1, e); this.controls.target.lerpVectors(this.anim.c0, this.anim.c1, e);
      if (k >= 1) this.anim = null;
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
  /** rebuild the model from the robot's numbers */
  update(r) {
    this.r = r;
    this.robotG.clear();
    const b = r.body, w = r.wheels, H = mm(heightOf(r)), rad = mm(w.diameter_mm / 2), bottom = rad * 0.45;
    const L = mm(b.front_mm + b.rear_mm), W = mm(b.width_mm);
    const body = new T.Mesh(new T.BoxGeometry(L, W, H), new T.MeshStandardMaterial({ color: css('--robot') || '#0f6e52', transparent: true, opacity: 0.55, roughness: 0.7 }));
    body.position.set(mm(b.front_mm) - L / 2, 0, bottom + H / 2); this.robotG.add(body); this.bodyMesh = body;
    const edges = new T.LineSegments(new T.EdgesGeometry(body.geometry), new T.LineBasicMaterial({ color: 0x0b5741 }));
    edges.position.copy(body.position); this.robotG.add(edges);
    // front arrow on the top
    const arrow = new T.Mesh(new T.ConeGeometry(mm(8), mm(18), 3), new T.MeshBasicMaterial({ color: 0xffffff }));
    arrow.rotation.z = -Math.PI / 2; arrow.position.set(mm(b.front_mm) - mm(22), 0, bottom + H + mm(1)); arrow.scale.z = 0.2; this.robotG.add(arrow);
    this.wheelMeshes = [];
    const xs = r.drive === '4wd' ? [w.x_mm + w.wheelbase_mm / 2, w.x_mm - w.wheelbase_mm / 2] : [+w.x_mm || 0];
    for (const x of xs) for (const side of [1, -1]) {
      const wh = new T.Mesh(new T.CylinderGeometry(rad, rad, mm(w.width_mm), 32), new T.MeshStandardMaterial({ color: 0x2b3442, roughness: 0.9 }));
      wh.position.set(mm(x), side * mm(w.track_mm / 2), rad); this.robotG.add(wh); this.wheelMeshes.push(wh);
    }
    this.casterMesh = null;
    if (r.drive === '2wd' && r.caster.enabled) {
      const cr = mm(r.caster.diameter_mm / 2);
      const ball = new T.Mesh(new T.SphereGeometry(cr, 20, 14), new T.MeshStandardMaterial({ color: 0x9aa6b0, metalness: 0.6, roughness: 0.3 }));
      ball.position.set(mm(r.caster.x_mm), mm(r.caster.y_mm || 0), cr); this.robotG.add(ball); this.casterMesh = ball;
    }
    this.sensorMeshes = [];
    r.sensors.forEach((s, i) => {
      const col = PALETTE[i % PALETTE.length], a = s.angle_deg * Math.PI / 180;
      const g = new T.Group(); g.position.set(mm(s.x_mm), mm(s.y_mm), bottom + H + mm(5)); g.rotation.z = a;
      g.add(new T.Mesh(new T.BoxGeometry(mm(8), mm(14), mm(10)), new T.MeshStandardMaterial({ color: col })));
      const ray = lineOf([[0, 0, 0], [Math.min(250, s.max_mm || 250), 0, 0]], col, true); ray.material.depthTest = true; g.add(ray);
      this.robotG.add(g); this.sensorMeshes.push(g);
    });
    // origin: the middle of the drive axle
    const o = new T.Mesh(new T.SphereGeometry(mm(3), 12, 8), new T.MeshBasicMaterial({ color: 0xc8352e }));
    o.position.set(0, 0, mm(1)); this.robotG.add(o);
    if (this.current) this.show(this.current, false);
  }
  /** the measuring line for one value (null clears) */
  show(path, turn = true) {
    this.current = path;
    this.measureG.clear();
    [this.bodyMesh, ...(this.wheelMeshes || []), this.casterMesh].filter(Boolean).forEach((m) => m.material.emissive && m.material.emissive.setHex(0));
    if (!path || !this.r) return;
    const d = measureOf(path, this.r); if (!d) return;
    const hot = 0xff1a1a, r = this.r, H = heightOf(r), top = r.wheels.diameter_mm / 2 * 0.45 + H;
    const hi = (m) => { if (m && m.material.emissive) m.material.emissive.setHex(0x553300); };
    if (d.part === 'body') hi(this.bodyMesh); if (d.part === 'wheel') this.wheelMeshes.forEach(hi); if (d.part === 'caster') hi(this.casterMesh);
    if (d.sensor !== undefined) this.sensorMeshes[d.sensor].children[0].material.emissive.setHex(0x553300);
    // reference lines: the axle line (x is measured from it), the centre line (y is measured from it)
    if (d.axle) this.measureG.add(lineOf([[0, -r.wheels.track_mm / 2 - 40, top + 3], [0, r.wheels.track_mm / 2 + 40, top + 3]], 0xc8352e, true));
    if (d.center) this.measureG.add(lineOf([[-r.body.rear_mm - 30, 0, top + 3], [r.body.front_mm + 40, 0, top + 3]], 0xc8352e, true));
    const val = (path.split('.').reduce((o, k) => (o == null ? o : o[k]), r));
    let mid;
    if (d.arc) {
      const [x, y, z] = d.arc.at, R = 45, n = 24, deg = d.arc.deg || 0, pts = [];
      for (let k = 0; k <= n; k++) { const a = deg * Math.PI / 180 * k / n; pts.push([x + R * Math.cos(a), y + R * Math.sin(a), z]); }
      this.measureG.add(lineOf([[x, y, z], [x + R + 25, y, z]], 0xc8352e, true), lineOf(pts, hot), laserDot(pts[0]), laserDot(pts[pts.length - 1]));
      const am = deg * Math.PI / 360; mid = [x + (R + 18) * Math.cos(am), y + (R + 18) * Math.sin(am), z];
    } else {
      const [a, b] = [d.a, d.b], dir = new T.Vector3(...b).sub(new T.Vector3(...a)), len = dir.length() || 1;
      // end ticks across the line
      const across = Math.abs(dir.z) > 0.5 * len ? [6, 0, 0] : (Math.abs(dir.x) > Math.abs(dir.y) ? [0, 6, 0] : [6, 0, 0]);
      const tick = (p) => lineOf([[p[0] - across[0], p[1] - across[1], p[2] - across[2]], [p[0] + across[0], p[1] + across[1], p[2] + across[2]]], hot);
      this.measureG.add(lineOf([a, b], hot), tick(a), tick(b), laserDot(a), laserDot(b));
      mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2 + 12];
    }
    const sp = label(`${Number.isFinite(+val) ? +(+val).toFixed(1) : '?'}${/angle/.test(path) ? '°' : ' mm'}`, '#ff1a1a');
    sp.position.set(...mid.map(mm)); this.measureG.add(sp);
    if (turn) this.look(d.view, mid);
  }
  /** move the camera to look from the top / the side / the front */
  look(view, at) {
    const c = new T.Vector3(...(at || [0, 0, 0]).map(mm)); c.z = Math.max(0.02, c.z * 0.6);
    const D = 0.42, p = view === 'side' ? new T.Vector3(c.x, c.y - D, c.z + 0.06) : view === 'front' ? new T.Vector3(c.x + D, c.y - 0.04, c.z + 0.08) : new T.Vector3(c.x - 0.02, c.y - 0.06, c.z + D);
    this.anim = { t0: performance.now(), p0: this.camera.position.clone(), p1: p, c0: this.controls.target.clone(), c1: c };
  }
}
export const handHint = () => t('builder.hand_hint');
