// train-worker.js - evaluates candidates in a Web Worker (one engine instance per training corridor).
import { loadEngine } from './engine.js';
import { trackSet, episodeConfig } from './train-core.js';

let E, sims = [], ctx = null;
self.onmessage = async (ev) => {
  const m = ev.data;
  if (m.type === 'init') {
    E = await loadEngine();
    for (const s of sims) s.destroy();
    ctx = m; sims = trackSet(m.env).map((tr) => {
      const s = E.create();
      s.walls(tr.walls); s.centerline(tr.center); s.finish(true, ...tr.finish); s.spawn(...tr.spawn);
      return s;
    });
    self.postMessage({ type: 'ready' });
  } else if (m.type === 'eval') {
    // m.cand, m.noise, m.episodes, m.gen, m.seed -> mean score and statistics over all corridors x episodes
    let score = 0, n = 0, crashes = 0, finishes = 0, ce = 0, he = 0, t = 0;
    for (const s of sims) {
      s.config(episodeConfig(ctx.robot, ctx.env, m.cand, m.noise));
      for (let e = 0; e < m.episodes; e++) {
        const r = s.evaluate(m.seed, m.gen, e);
        score += r.score; n++; crashes += r.crashed; finishes += r.finished; ce += r.meanCenter; he += r.meanHeading; t += r.time;
      }
    }
    self.postMessage({ type: 'result', id: m.id, score: score / n, crash: crashes / n, finish: finishes / n, ce: ce / n * 1000, he: he / n * 180 / Math.PI, time: t / n, n });
  }
};
