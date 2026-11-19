// algorithms.js - example maze algorithms. Interface (also for your own code):
//   create(ctx, kit) -> { step(obs) -> {left, right} (wheel speeds, m/s), map?, pose?, visited? }
//   ctx: {rows, cols, cell, thick, start: {r, c, heading}, startDir, goal: [[r, c]...], sensors: [{x, y, angle, min, max}],
//         track (m), vmax (m/s), dwellTicks, wallKp / wallKd (side-wall centring while driving)}
//   obs: {t, dt, dists: [m...], enc: {l, r} (m/s), encDist: {l, r} (m, since the start)}
// Exposing `map` (a kit.Mapper) and `pose` lets the page draw what the robot believes.
import { CellRobot, floodDistances } from './mazekit.js';

const turnOrder = (h) => [(h + 3) % 4, h, (h + 1) % 4, (h + 2) % 4];  // left, straight, right, back

export const ALGORITHMS = {
  wall_follower: {
    create(ctx) {
      // left-hand rule: take the first way that is not a known wall, in the order left, straight, right, back
      return new CellRobot(ctx, (cell, h, map) => turnOrder(h).find((d) => map.wall(cell[0], cell[1], d) !== 1));
    },
  },
  flood_fill: {
    create(ctx) {
      // assume unknown walls are open, flood distances from the goal, move to the neighbour closest to it;
      // re-flood after every cell (the map grows as the robot drives)
      return new CellRobot(ctx, (cell, h, map) => {
        const D = floodDistances(map, ctx.goal);
        let best = null, bd = Infinity;
        for (const d of [h, (h + 3) % 4, (h + 1) % 4, (h + 2) % 4]) {
          if (map.wall(cell[0], cell[1], d) === 1) continue;
          const ny = cell[0] + [1, 0, -1, 0][d], nx = cell[1] + [0, 1, 0, -1][d];
          if (D[ny] && D[ny][nx] < bd) { bd = D[ny][nx]; best = d; }
        }
        return best;
      });
    },
  },
  dfs: {
    create(ctx) {
      // depth-first exploration with mapping: go to an unvisited open neighbour, otherwise go back the way you came
      const stack = [];
      const robot = new CellRobot(ctx, (cell, h, map, self) => {
        for (const d of turnOrder(h)) {
          if (map.wall(cell[0], cell[1], d) === 1) continue;
          const n = [cell[0] + [1, 0, -1, 0][d], cell[1] + [0, 1, 0, -1][d]];
          if (!self.visited.has(n + '')) { stack.push((d + 2) % 4); return d; }
        }
        return stack.length ? stack.pop() : null;  // backtrack (null = everything explored)
      });
      return robot;
    },
  },
};

export const CUSTOM_TEMPLATE = `// Your algorithm. kit = mazekit (CellRobot, Mapper, Odometry, Motion, floodDistances, DIRS).
// Return an object with step(obs) -> {left, right}. Here: right-hand wall follower on top of kit.CellRobot.
function create(ctx, kit) {
  return new kit.CellRobot(ctx, (cell, h, map) => {
    for (const d of [(h + 1) % 4, h, (h + 3) % 4, (h + 2) % 4])   // right, straight, left, back
      if (map.wall(cell[0], cell[1], d) !== 1) return d;
    return null;
  });
}`;
