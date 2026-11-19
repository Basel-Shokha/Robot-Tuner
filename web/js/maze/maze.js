// maze.js - maze generation and construction.
//
// Layout: rows x cols cells of size `cell` (m); cell (r, c) has its centre at ((c + .5) cell, (r + .5) cell),
// r = 0 is the bottom row. Edges: H[r][c] = wall below row r (r = 0..rows), V[r][c] = wall left of column c
// (c = 0..cols). Posts are the lattice points P[r][c] (r = 0..rows, c = 0..cols).
//
// Realistic construction (not a perfect maze jittered afterwards): the maze is BUILT WALL BY WALL, like by
// hand. The first post is placed exactly; then walls are added in breadth-first order through the wall graph.
// A wall starting at an already-placed post is laid in its nominal direction with a small random angle
// error and length error, which places its far post; a wall between two already-placed posts is fitted
// between them (with a small attachment error at each end). Errors therefore ACCUMULATE along the build
// order, like in a real hand-built maze, while at the default size the maze still looks perfect.
import { rng, gauss } from '../corridor.js';

export function defaultMazeSettings() {
  return { kind: 'thick', cols: 6, rows: 6, cell_mm: 300, wall_mm: 12, err_pos_mm: 1.0, err_angle_deg: 0.3, seed: 1, goal: 'corner', braid: 0 };
}

/** perfect maze by recursive backtracker (iterative), then `braid` fraction of extra openings (loops) */
export function generateGrid(rows, cols, seed, braid = 0) {
  const r = rng(seed);
  const H = Array.from({ length: rows + 1 }, () => new Array(cols).fill(1));
  const V = Array.from({ length: rows }, () => new Array(cols + 1).fill(1));
  const seen = Array.from({ length: rows }, () => new Array(cols).fill(false));
  const st = [[0, 0]]; seen[0][0] = true;
  while (st.length) {
    const [y, x] = st[st.length - 1];
    const nb = [[y + 1, x, 'N'], [y - 1, x, 'S'], [y, x + 1, 'E'], [y, x - 1, 'W']].filter(([a, b]) => a >= 0 && a < rows && b >= 0 && b < cols && !seen[a][b]);
    if (!nb.length) { st.pop(); continue; }
    const [a, b, d] = nb[Math.floor(r() * nb.length)];
    if (d === 'N') H[y + 1][x] = 0; else if (d === 'S') H[y][x] = 0; else if (d === 'E') V[y][x + 1] = 0; else V[y][x] = 0;
    seen[a][b] = true; st.push([a, b]);
  }
  for (let y = 1; y < rows; y++) for (let x = 0; x < cols; x++) if (H[y][x] && r() < braid) H[y][x] = 0;
  for (let y = 0; y < rows; y++) for (let x = 1; x < cols; x++) if (V[y][x] && r() < braid) V[y][x] = 0;
  return { rows, cols, H, V };
}

export function emptyGrid(rows, cols) {
  const H = Array.from({ length: rows + 1 }, (_, y) => new Array(cols).fill(y === 0 || y === rows ? 1 : 0));
  const V = Array.from({ length: rows }, () => Array.from({ length: cols + 1 }, (_, x) => (x === 0 || x === cols ? 1 : 0)));
  return { rows, cols, H, V };
}

/** list of wall edges as post pairs [[r1,c1],[r2,c2], kind, i, j] */
function edges(g) {
  const out = [];
  for (let y = 0; y <= g.rows; y++) for (let x = 0; x < g.cols; x++) if (g.H[y][x]) out.push([[y, x], [y, x + 1], 'H', y, x]);
  for (let y = 0; y < g.rows; y++) for (let x = 0; x <= g.cols; x++) if (g.V[y][x]) out.push([[y, x], [y + 1, x], 'V', y, x]);
  return out;
}

/** Build the physical maze wall by wall. Returns {walls (flat segments), posts, built: [{a, b, kind}], ...} */
export function construct(g, s) {
  const cell = s.cell_mm / 1000, ep = s.err_pos_mm / 1000, ea = s.err_angle_deg * Math.PI / 180, r = rng(s.seed * 31 + 7);
  const P = Array.from({ length: g.rows + 1 }, () => new Array(g.cols + 1).fill(null));
  const E = edges(g);
  const byPost = new Map();
  const key = (p) => p[0] * 10000 + p[1];
  E.forEach((e, i) => { for (const p of [e[0], e[1]]) { const k = key(p); if (!byPost.has(k)) byPost.set(k, []); byPost.get(k).push(i); } });
  const done = new Array(E.length).fill(false);
  const built = [];
  const queue = [];
  const place = (p, xy) => { P[p[0]][p[1]] = xy; queue.push(p); };
  // breadth-first from the first post that has a wall (normally the corner)
  const startPost = E.length ? E[0][0] : [0, 0];
  place(startPost, [startPost[1] * cell, startPost[0] * cell]);
  const lay = () => {
    while (queue.length) {
      const p = queue.shift();
      for (const i of byPost.get(key(p)) || []) {
        if (done[i]) continue;
        done[i] = true;
        const e = E[i], q = key(e[0]) === key(p) ? e[1] : e[0];
        const A = P[p[0]][p[1]];
        let B = P[q[0]][q[1]];
        const end = (xy) => [xy[0] + gauss(r) * ep / 3, xy[1] + gauss(r) * ep / 3];
        if (!B) {
          const nominal = Math.atan2(q[0] - p[0], q[1] - p[1]);
          const a = nominal + gauss(r) * ea, L = cell * (1 + gauss(r) * ep / cell);
          B = [A[0] + Math.cos(a) * L + gauss(r) * ep / 3, A[1] + Math.sin(a) * L + gauss(r) * ep / 3];
          place(q, B);
          built.push({ a: end(A), b: B, kind: e[2], i: e[3], j: e[4] });
        } else built.push({ a: end(A), b: end(B), kind: e[2], i: e[3], j: e[4] });
      }
    }
  };
  lay();
  // walls not connected to the first post (only possible in drawn mazes): start a new group nearby
  for (let i = 0; i < E.length; i++) if (!done[i]) {
    const p = E[i][0];
    if (!P[p[0]][p[1]]) place(p, nearestPlaced(P, p, cell));
    lay();
  }
  // posts without walls (thick mazes have them): placed relative to a placed neighbour
  for (let y = 0; y <= g.rows; y++) for (let x = 0; x <= g.cols; x++) if (!P[y][x]) P[y][x] = nearestPlaced(P, [y, x], cell, r, ep);
  // wall geometry
  const walls = [];
  const T = s.kind === 'thick' ? s.wall_mm / 1000 : 0;
  for (const w of built) {
    if (T <= 0) { walls.push(w.a[0], w.a[1], w.b[0], w.b[1]); continue; }
    const dx = w.b[0] - w.a[0], dy = w.b[1] - w.a[1], L = Math.hypot(dx, dy), ux = dx / L, uy = dy / L, nx = -uy * T / 2, ny = ux * T / 2;
    const a = [w.a[0] - ux * T / 2, w.a[1] - uy * T / 2], b = [w.b[0] + ux * T / 2, w.b[1] + uy * T / 2];
    const c = [[a[0] + nx, a[1] + ny], [b[0] + nx, b[1] + ny], [b[0] - nx, b[1] - ny], [a[0] - nx, a[1] - ny]];
    for (let k = 0; k < 4; k++) walls.push(c[k][0], c[k][1], c[(k + 1) % 4][0], c[(k + 1) % 4][1]);
  }
  return { walls, posts: P, built, cell, thickness: T };
}
function nearestPlaced(P, p, cell, r = null, ep = 0) {
  for (let d = 1; d < 100; d++) for (let dy = -d; dy <= d; dy++) for (let dx = -d; dx <= d; dx++) {
    const y = p[0] + dy, x = p[1] + dx;
    if (P[y] && P[y][x]) return [P[y][x][0] - dx * cell + (r ? gauss(r) * ep : 0), P[y][x][1] - dy * cell + (r ? gauss(r) * ep : 0)];
  }
  return [p[1] * cell, p[0] * cell];
}

export function goalCells(g, mode) {
  if (mode === 'center' && g.rows >= 2 && g.cols >= 2) {
    const y = Math.floor((g.rows - 1) / 2), x = Math.floor((g.cols - 1) / 2);
    return g.rows % 2 || g.cols % 2 ? [[Math.floor(g.rows / 2), Math.floor(g.cols / 2)]] : [[y, x], [y + 1, x], [y, x + 1], [y + 1, x + 1]];
  }
  return [[g.rows - 1, g.cols - 1]];
}
/** real centre of a cell = mean of its four (built) posts */
export function cellCenter(m, y, x) {
  const P = m.posts, q = [P[y][x], P[y][x + 1], P[y + 1][x], P[y + 1][x + 1]];
  return [(q[0][0] + q[1][0] + q[2][0] + q[3][0]) / 4, (q[0][1] + q[1][1] + q[2][1] + q[3][1]) / 4];
}
/** cell the point lies in (nominal grid) */
export function cellOf(cell, x, y) { return [Math.floor(y / cell), Math.floor(x / cell)]; }

/** smallest cell (mm) the robot can turn around in, with a 2 cm margin */
export function minCellMm(robot) {
  const b = robot.body, w = robot.wheels;
  const r = Math.max(Math.hypot(b.front_mm, b.width_mm / 2), Math.hypot(b.rear_mm, b.width_mm / 2), Math.hypot(Math.abs(w.x_mm) + w.diameter_mm / 2 + (robot.drive === '4wd' ? w.wheelbase_mm / 2 : 0), w.track_mm / 2 + w.width_mm / 2));
  return Math.ceil((2 * r + 40) / 10) * 10;
}
