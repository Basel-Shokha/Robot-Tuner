import { test } from 'node:test';
import assert from 'node:assert/strict';
import { generateGrid, construct, defaultMazeSettings } from '../../web/js/maze/maze.js';

test('generated mazes are perfect (every cell reachable, no loops)', () => {
  for (const seed of [1, 2, 3, 99]) {
    const g = generateGrid(7, 9, seed);
    let open = 0;
    for (let y = 1; y < g.rows; y++) for (let x = 0; x < g.cols; x++) open += 1 - g.H[y][x];
    for (let y = 0; y < g.rows; y++) for (let x = 1; x < g.cols; x++) open += 1 - g.V[y][x];
    assert.equal(open, g.rows * g.cols - 1, 'a spanning tree has cells - 1 openings');
    const seen = new Set(['0,0']), st = [[0, 0]];
    while (st.length) {
      const [y, x] = st.pop();
      const nb = [];
      if (y + 1 < g.rows && !g.H[y + 1][x]) nb.push([y + 1, x]); if (y > 0 && !g.H[y][x]) nb.push([y - 1, x]);
      if (x + 1 < g.cols && !g.V[y][x + 1]) nb.push([y, x + 1]); if (x > 0 && !g.V[y][x]) nb.push([y, x - 1]);
      for (const n of nb) if (!seen.has(n + '')) { seen.add(n + ''); st.push(n); }
    }
    assert.equal(seen.size, g.rows * g.cols);
  }
});

test('construction is deterministic, errors accumulate, default looks perfect', () => {
  const s = { ...defaultMazeSettings(), cols: 8, rows: 8 };
  const g = generateGrid(8, 8, 5);
  const a = construct(g, s), b = construct(g, s);
  assert.deepEqual(a.walls, b.walls);
  const P = a.posts, cell = a.cell;
  const dev = (y, x) => Math.hypot(P[y][x][0] - x * cell, P[y][x][1] - y * cell);
  assert.ok(dev(8, 8) < 0.03, 'default: far corner within 3 cm');
  // bigger errors: deviation grows with build distance from the first post (accumulation)
  const big = construct(g, { ...s, err_pos_mm: 5, err_angle_deg: 2 });
  let near = 0, far = 0;
  const d2 = (y, x) => Math.hypot(big.posts[y][x][0] - x * cell, big.posts[y][x][1] - y * cell);
  for (let y = 0; y <= 2; y++) for (let x = 0; x <= 2; x++) near += d2(y, x) / 9;
  for (let y = 6; y <= 8; y++) for (let x = 6; x <= 8; x++) far += d2(y, x) / 9;
  assert.ok(far > near, `far ${far} > near ${near}`);
});
