// mazekit.js - building blocks for maze algorithms that run on the simulated robot.
// An algorithm only gets what a real robot gets: distance readings, encoder speeds / distances, time.
//   Odometry   pose from the wheel encoders (it drifts, like on the real robot)
//   Mapper     the robot's own map: every reading adds evidence "wall here" / "free space here" to grid edges
//   Motion     drive to a cell centre (turn in place, then drive), wheel commands in m/s
//   CellRobot  ties them together: implement decide(cell, heading, map) -> neighbour cell, it does the rest
export const DIRS = [[1, 0], [0, 1], [-1, 0], [0, -1]];  // N, E, S, W as [dr, dc]; heading index 0..3 (N = +y)
export const wrap = (a) => { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; };

export class Odometry {
  constructor(x, y, th, track) { this.x = x; this.y = y; this.th = th; this.track = track; this.l = 0; this.r = 0; }
  update(encDistL, encDistR) {
    const dl = encDistL - this.l, dr = encDistR - this.r; this.l = encDistL; this.r = encDistR;
    const ds = (dl + dr) / 2, dth = (dr - dl) / this.track;
    this.x += ds * Math.cos(this.th + dth / 2); this.y += ds * Math.sin(this.th + dth / 2); this.th = wrap(this.th + dth);
  }
}

export class Mapper {
  /** rows, cols, cell (m), wall thickness (m) */
  constructor(rows, cols, cell, thick = 0) {
    Object.assign(this, { rows, cols, cell, thick });
    this.H = Array.from({ length: rows + 1 }, (_, y) => Array.from({ length: cols }, () => ({ w: y === 0 || y === rows ? 99 : 0, o: 0 })));
    this.V = Array.from({ length: rows }, () => Array.from({ length: cols + 1 }, (_, x) => ({ w: x === 0 || x === cols ? 99 : 0, o: 0 })));
    this.hgain = 0.5; this.lines = [];  // per sensor: recent hit points on one grid line, for the heading correction
  }
  // Heading correction: the hit points of one sensor on one wall should lie on a grid line. If they form a
  // slanted line in the believed frame, the believed heading is off by that slant: turn it back.
  _line(i, kind, k, x, y, pose, gain) {
    let b = this.lines[i];
    if (!b || b.kind !== kind || b.k !== k) b = this.lines[i] = { kind, k, p: [] };
    b.p.push(kind === 'H' ? [x, y] : [y, x]);
    if (b.p.length > 14) b.p.shift();
    if (b.p.length < 8) return;
    const n = b.p.length, mx = b.p.reduce((s, q) => s + q[0], 0) / n, my = b.p.reduce((s, q) => s + q[1], 0) / n;
    let sxx = 0, sxy = 0;
    for (const q of b.p) { sxx += (q[0] - mx) ** 2; sxy += (q[0] - mx) * (q[1] - my); }
    if (sxx < 0.04 * 0.04 * n / 12) return;  // the points span less than ~4 cm: no reliable direction
    const ang = Math.atan(sxy / sxx) * (kind === 'H' ? 1 : -1);
    if (Math.abs(ang) < 0.35) pose.th = wrap(pose.th - gain * ang);
    b.p = [];
  }
  /** 1 wall, 0 open, -1 unknown */
  state(e) { if (!e) return 1; if (e.w >= 3 && e.w > 1.5 * e.o) return 1; if (e.o >= 3 && e.o > 1.5 * e.w) return 0; return -1; }
  /** edge between cell (y,x) and its neighbour in direction d (0 N, 1 E, 2 S, 3 W) */
  edge(y, x, d) { return d === 0 ? (this.H[y + 1] || [])[x] : d === 2 ? (this.H[y] || [])[x] : d === 1 ? (this.V[y] || [])[x + 1] : (this.V[y] || [])[x]; }
  wall(y, x, d) { return this.state(this.edge(y, x, d)); }
  /** Direct observation at a cell centre, facing direction d with the front sensor: wall or open.
   *  Also corrects the position along that axis from the distance to the wall (like a real micromouse). */
  observeFront(pose, y, x, d, reading, sensor) {
    if (globalThis.RT_DEBUG && typeof console !== "undefined") console.log("obs", y, x, d, reading.toFixed(3), "exp", (this.cell / 2 - this.thick / 2 - sensor.x).toFixed(3), "pose", pose.x.toFixed(3), pose.y.toFixed(3), pose.th.toFixed(2));
    const e = this.edge(y, x, d); if (!e) return;
    const expect = this.cell / 2 - this.thick / 2 - sensor.x;  // sensor to wall face, robot centred
    if (reading < expect + this.cell * 0.4) {
      e.w += 5;
      const err = reading - expect;  // > 0: wall farther than expected = robot further back
      if (Math.abs(err) < 0.03) {
        if (d === 0) pose.y = (y + 0.5) * this.cell - err; else if (d === 2) pose.y = (y + 0.5) * this.cell + err;
        else if (d === 1) pose.x = (x + 0.5) * this.cell - err; else pose.x = (x + 0.5) * this.cell + err;
      }
    } else e.o += 5;
  }
  markOpen(y, x, d) { const e = this.edge(y, x, d); if (e && e.w < 99) e.o += 5; }
  /** pose: Odometry; sensors: [{x, y, angle_rad, min, max}]; dists (m). Returns lateral pose correction. */
  update(pose, sensors, dists, correct = 0.1, evidence = true) {
    const c = this.cell, tol = c * 0.12 + this.thick / 2;
    const co = Math.cos(pose.th), si = Math.sin(pose.th);
    for (let i = 0; i < sensors.length; i++) {
      const s = sensors[i], d = dists[i];
      if (!(d > s.min * 1.05)) continue;
      const ox = pose.x + co * s.x - si * s.y, oy = pose.y + si * s.x + co * s.y, a = pose.th + s.angle;
      const ux = Math.cos(a), uy = Math.sin(a);
      const hit = d < s.max * 0.95;
      const free = hit ? d - tol : s.max * 0.9;
      // free space: grid lines crossed by the ray (away from the posts) are open
      let py = Math.floor(oy / c), px = Math.floor(ox / c);
      for (let t = c / 10; t < free; t += c / 10) {
        const qx = ox + ux * t, qy = oy + uy * t, cy = Math.floor(qy / c), cx = Math.floor(qx / c);
        if (!evidence) break;
        if (cy !== py && cx === px) { const fx = qx / c - cx; if (fx > 0.2 && fx < 0.8) { const e = (this.H[Math.max(cy, py)] || [])[cx]; if (e) e.o++; } }
        else if (cx !== px && cy === py) { const fy = qy / c - cy; if (fy > 0.2 && fy < 0.8) { const e = (this.V[cy] || [])[Math.max(cx, px)]; if (e) e.w !== 99 && e.o++; } }
        py = cy; px = cx;
      }
      if (!hit) continue;
      const hx = ox + ux * d, hy = oy + uy * d;
      const ky = Math.round(hy / c), kx = Math.round(hx / c);
      const dyH = Math.abs(hy - ky * c), dxV = Math.abs(hx - kx * c);
      const colH = Math.floor(hx / c), rowV = Math.floor(hy / c);
      if (dyH < tol && dyH <= dxV) {
        const e = (this.H[ky] || [])[colH];
        if (e) { if (evidence) e.w++; this._line(i, 'H', ky, hx, hy, pose, this.hgain); if (correct && this.state(e) === 1 && Math.abs(uy) > 0.5) pose.y += correct * (ky * c - (hy > ky * c ? -1 : 1) * this.thick / 2 - hy); }
      } else if (dxV < tol) {
        const e = (this.V[rowV] || [])[kx];
        if (e) { if (evidence) e.w++; this._line(i, 'V', kx, hx, hy, pose, this.hgain); if (correct && this.state(e) === 1 && Math.abs(ux) > 0.5) pose.x += correct * (kx * c - (hx > kx * c ? -1 : 1) * this.thick / 2 - hx); }
      }
    }
  }
}

export class Motion {
  constructor(track, vmax, accel = 0.4) { this.track = track; this.vmax = vmax; this.accel = accel; this.target = null; this.out = { left: 0, right: 0 }; }
  /** wheel commands change by at most accel * dt per tick (abrupt steps make the wheels slip) */
  ramp(c, dt) {
    const m = this.accel * dt, o = this.out;
    o.left += Math.max(-m, Math.min(m, c.left - o.left)); o.right += Math.max(-m, Math.min(m, c.right - o.right));
    return { left: o.left, right: o.right };
  }
  go(x, y, from) { this.target = [x, y]; this.from = from; this.phase = 'turn'; this.face = null; }
  /** turn in place to a heading (rad) */
  turnTo(th) { this.face = th; this.target = null; }
  /** turn rate for a remaining angle b: as fast as allowed, but able to stop in time (ramped wheels) */
  turnRate(b) {
    const wmax = 1.6 * this.vmax / this.track, alpha = 0.8 * 2 * this.accel / this.track;
    return Math.sign(b) * Math.min(wmax, Math.sqrt(2 * alpha * Math.abs(b)), 6 * Math.abs(b) + 0.15);
  }
  get busy() { return !!this.target || this.face !== null && this.face !== undefined; }
  /** wheel commands {left, right} (m/s) toward the target; clears the target on arrival */
  cmd(pose) {
    if (this.face !== null && this.face !== undefined) {
      const b = wrap(this.face - pose.th);
      if (Math.abs(b) < 0.03) { this.face = null; return { left: 0, right: 0 }; }
      const h = this.turnRate(b) * this.track / 2;
      return { left: -h, right: h };
    }
    if (!this.target) return { left: 0, right: 0 };
    const dx = this.target[0] - pose.x, dy = this.target[1] - pose.y, dist = Math.hypot(dx, dy);
    const b = wrap(Math.atan2(dy, dx) - pose.th);
    const wmax = 1.6 * this.vmax / this.track;
    let v = 0, w = 0;
    // arrived: close to the target, or (driving) the remaining distance ALONG the cell-to-cell axis is ~0
    let along = dist;
    if (this.from) { const ax = this.target[0] - this.from[0], ay = this.target[1] - this.from[1], L = Math.hypot(ax, ay) || 1; along = (dx * ax + dy * ay) / L; }
    if (dist < 0.012 || (this.phase === 'drive' && along < 0.008)) { this.target = null; return { left: 0, right: 0 }; }
    if (this.phase === 'turn') {
      w = this.turnRate(b);
      if (Math.abs(b) < 0.04) { this.phase = 'drive'; w = 0; }
    } else {
      v = Math.min(this.vmax, 1.5 * Math.max(0, along) + 0.03) * Math.max(0, Math.cos(b));
      w = Math.max(-wmax, Math.min(wmax, 4 * b));
    }
    const h = w * this.track / 2;
    return { left: v - h, right: v + h };
  }
}

/** Grid robot: dwell at each cell centre to map, then call decide() and drive to the chosen neighbour. */
export class CellRobot {
  constructor(ctx, decide) {
    this.ctx = ctx; this.decide = decide;
    const c = ctx.cell;
    this.pose = new Odometry((ctx.start.c + 0.5) * c, (ctx.start.r + 0.5) * c, ctx.start.heading, ctx.track);
    this.map = new Mapper(ctx.rows, ctx.cols, c, ctx.thick || 0);
    this.motion = new Motion(ctx.track, ctx.vmax);
    this.cell = [ctx.start.r, ctx.start.c]; this.heading = ctx.startDir; this.dwell = 0; this.visited = new Set([this.cell + '']);
    this.done = false; this.path = []; this.pending = null; this.rayMapping = false;
    // the most forward-looking sensor reads the walls; the most left / right looking ones centre the robot
    this.front = 0; this.left = 0; this.right = 0;
    ctx.sensors.forEach((q, i) => {
      if (Math.abs(q.angle) < Math.abs(ctx.sensors[this.front].angle)) this.front = i;
      if (q.angle > ctx.sensors[this.left].angle) this.left = i;
      if (q.angle < ctx.sensors[this.right].angle) this.right = i;
    });
    if (ctx.sensors[this.left].angle < 0.3 || ctx.sensors[this.right].angle > -0.3) this.left = this.right = -1;
  }
  /** Between cells: steer to the middle between the side walls (like the corridor PID) and re-align the
   *  believed heading / lateral position with the grid while both side walls are seen. */
  wallAlign(obs, c) {
    const dl = obs.dists[this.left], dr = obs.dists[this.right];
    if (this.left < 0) return;
    const sl = this.ctx.sensors[this.left], sr = this.ctx.sensors[this.right];
    const side = (q) => (this.ctx.cell / 2 - (this.ctx.thick || 0) / 2 - Math.abs(q.y)) / Math.max(0.3, Math.abs(Math.sin(q.angle)));
    if (!(dl < 1.35 * side(sl) && dr < 1.35 * side(sr))) { this.prevE = undefined; return; }  // both side walls present
    const e = Math.max(-0.05, Math.min(0.05, dl - dr));
    // PD on the side-wall difference (the corridor controller); the walls, not the belief, steer here
    const de = this.prevE === undefined ? 0 : (e - this.prevE) / (obs.dt || 0.025);
    this.prevE = e;
    const v = (c.left + c.right) / 2;
    if (v < 0.3 * this.ctx.vmax) return;  // only while cruising; steering scales with speed (no turning on the spot)
    const h = Math.max(-0.3 * v, Math.min(0.3 * v, (this.ctx.wallKp * e + this.ctx.wallKd * de) * v / this.ctx.vmax));
    c.left = v - h; c.right = v + h;
    const card = Math.PI / 2 - this.heading * Math.PI / 2;
    if (Math.abs(e) < 0.02 && Math.abs(de) < 0.05) {
      this.pose.th = wrap(this.pose.th + 0.1 * wrap(card - this.pose.th));
      if (this.heading % 2 === 0) this.pose.x += 0.05 * ((this.cell[1] + 0.5) * this.ctx.cell - this.pose.x);
      else this.pose.y += 0.05 * ((this.cell[0] + 0.5) * this.ctx.cell - this.pose.y);
    }
  }
  step(obs) {
    this.pose.update(obs.encDist.l, obs.encDist.r);
    // wall hits only re-align the heading (rays add no wall evidence unless rayMapping is on)
    if (this.headingFromWalls && this.motion.phase === 'drive') this.map.update(this.pose, this.ctx.sensors, obs.dists, 0, !!this.rayMapping);
    if (this.done) return { left: 0, right: 0 };
    if (this.motion.busy) {
      const c = this.motion.cmd(this.pose);
      if (this.motion.phase === 'drive' && this.motion.target) this.wallAlign(obs, c);
      return this.motion.ramp(c, obs.dt || 0.025);
    }
    const rest = this.motion.ramp({ left: 0, right: 0 }, obs.dt || 0.025);
    if (Math.abs(rest.left) + Math.abs(rest.right) > 1e-4) return rest;
    if (this.dwell < this.ctx.dwellTicks) { this.dwell++; return rest; }
    this.dwell = 0;
    // look around: face every edge of this cell that is still unknown and read the front sensor
    if (this.pending !== null) {
      this.map.observeFront(this.pose, this.cell[0], this.cell[1], this.pending, obs.dists[this.front], this.ctx.sensors[this.front]);
      this.pending = null;
    }
    for (let k = 0; k < 4; k++) {
      const d = (this.heading + [0, 3, 1, 2][k]) % 4;
      if (this.map.wall(this.cell[0], this.cell[1], d) === -1) {
        this.pending = d;
        this.motion.turnTo(Math.PI / 2 - d * Math.PI / 2);
        this.heading = d;
        return this.motion.ramp(this.motion.cmd(this.pose), obs.dt || 0.025);
      }
    }
    if (this.ctx.goal.some((g) => g[0] === this.cell[0] && g[1] === this.cell[1])) { this.done = true; return { left: 0, right: 0 }; }
    const d = this.decide(this.cell, this.heading, this.map, this);
    if (d === null || d === undefined) { this.done = true; return { left: 0, right: 0 }; }
    const n = [this.cell[0] + DIRS[d][0], this.cell[1] + DIRS[d][1]];
    this.map.markOpen(this.cell[0], this.cell[1], d);
    const from = [(this.cell[1] + 0.5) * this.ctx.cell, (this.cell[0] + 0.5) * this.ctx.cell];
    this.heading = d; this.cell = n; this.visited.add(n + ''); this.path.push(n);
    this.motion.go((n[1] + 0.5) * this.ctx.cell, (n[0] + 0.5) * this.ctx.cell, from);
    return this.motion.ramp(this.motion.cmd(this.pose), obs.dt || 0.025);
  }
}

/** breadth-first distances to the goal cells through edges that are not known walls (unknown = open) */
export function floodDistances(map, goal) {
  const D = Array.from({ length: map.rows }, () => new Array(map.cols).fill(Infinity));
  const q = [];
  for (const g of goal) { D[g[0]][g[1]] = 0; q.push(g); }
  while (q.length) {
    const [y, x] = q.shift();
    for (let d = 0; d < 4; d++) {
      const ny = y + DIRS[d][0], nx = x + DIRS[d][1];
      if (ny < 0 || nx < 0 || ny >= map.rows || nx >= map.cols || map.wall(y, x, d) === 1 || D[ny][nx] <= D[y][x] + 1) continue;
      D[ny][nx] = D[y][x] + 1; q.push([ny, nx]);
    }
  }
  return D;
}
