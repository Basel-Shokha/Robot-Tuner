// The WebAssembly build (web/wasm/rt_sim.wasm) must reproduce the native C++ build bit for bit:
// replays tests/golden/scenarios.txt through the JS API and compares with tests/golden/golden.txt, which
// tests/native/golden.cpp wrote from the native build (same C API, same call sequence).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { loadEngine } from '../web/js/engine.js';

function parse(text) {
  const lines = text.split('\n');
  const out = []; let s = null;
  for (let i = 0; i < lines.length; i++) {
    const t = lines[i].trim().split(/\s+/); const k = t[0];
    if (k === 'scenario') { s = { name: t[1], config: '', walls: [], center: [], fin: [0, 0, 0, 0] }; out.push(s); }
    else if (!s) continue;
    else if (k === 'seed') s.seed = +t[1];
    else if (k === 'ticks') s.ticks = +t[1];
    else if (k === 'external') s.external = +t[1];
    else if (k === 'sample_bias') s.bias = +t[1];
    else if (k === 'spawn') s.spawn = t.slice(1, 4).map(Number);
    else if (k === 'config') { while (lines[++i] !== 'end') s.config += lines[i] + '\n'; }
    else if (k === 'walls' || k === 'center') {
      const n = +t[1];
      for (let j = 0; j < n; j++) (k === 'walls' ? s.walls : s.center).push(...lines[++i].trim().split(/\s+/).map(Number));
    }
    else if (k === 'finish') { s.finish = +t[1]; s.fin = t.slice(2, 6).map(Number); }
    else if (k === 'evaluate') s.ev = t.slice(1, 4).map(Number);
  }
  return out;
}

function fnv(h, bytes) { for (const b of bytes) { h ^= b; h = Math.imul(h, 16777619) >>> 0; } return h >>> 0; }
const hex = (h) => h.toString(16).padStart(8, '0');

test('WASM build reproduces the native golden runs bit for bit', async () => {
  const E = await loadEngine();
  const scen = parse(readFileSync(new URL('./golden/scenarios.txt', import.meta.url), 'utf8'));
  const gold = readFileSync(new URL('./golden/golden.txt', import.meta.url), 'utf8').trim().split('\n');
  assert.equal(scen.length, gold.length);
  const idx = [1, 2, 3, 4, 5, 9, 10, 15, 24, 25, 27, 33, 34, 35, 36, 37];
  scen.forEach((s, n) => {
    const sim = E.create();
    sim.config(s.config);
    sim.walls(s.walls);
    if (s.center.length) sim.centerline(s.center);
    sim.finish(s.finish, ...s.fin);
    sim.spawn(...s.spawn);
    sim.reset(...s.spawn, s.seed, s.bias);
    sim.external(s.external);
    let h = 2166136261, k = 0;
    const f = new Float32Array(1), b = new Uint8Array(f.buffer);
    for (; k < s.ticks; k++) {
      if (s.external) {
        let vl = Math.fround(0.06 + 0.01 * (k % 7)); const vr = Math.fround(0.06 + 0.01 * ((k * 3) % 7));
        if (Math.floor(k / 40) % 3 === 2) vl = -vl;
        sim.cmd(vl, vr);
      }
      const alive = sim.step(1);
      const st = sim.stateRaw();
      for (const i of idx) { f[0] = st[i]; h = fnv(h, b); }
      if (!alive || st[7] !== 0) { k++; break; }
    }
    let out = `${s.name} ticks ${k} hash ${hex(h)}`;
    for (let e = 0; e < s.ev[2]; e++) {
      const r = sim.evaluate(s.ev[0], s.ev[1], e).raw;
      out += ` ep${e} ${hex(fnv(2166136261, new Uint8Array(r.buffer, r.byteOffset, 36)))}`;
    }
    sim.destroy();
    assert.equal(out, gold[n], `scenario ${s.name}`);
  });
});
