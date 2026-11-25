// sw.js - offline cache. Every file of the site is cached on first visit; afterwards the site works offline.
// Network first for pages and code (so updates arrive), cache first otherwise. Bump VERSION on release.
const VERSION = 'rt-v18';
const CORE = [
  './', 'index.html', 'builder.html', 'measure.html', 'corridor.html', 'playground.html', 'train.html', 'troubleshoot.html',
  'tutorial.html', 'maze.html', 'css/style.css', 'img/icon.svg', 'i18n/en.json', 'i18n/he.json', 'i18n/ar.json',
  'wasm/rt_sim.mjs', 'wasm/rt_sim.wasm', 'js/i18n.js', 'js/common.js', 'js/store.js', 'js/engine.js', 'js/draw.js', 'js/plot.js',
  'js/corridor.js', 'js/tracklib.js', 'js/simview.js', 'js/presets.js', 'js/train-core.js', 'js/train-worker.js', 'js/export.js', 'js/serial.js',
  'js/measure-steps.js', 'js/firmware-gen.js', 'js/builder.js', 'js/maze/maze.js', 'js/maze/algorithms.js', 'js/maze/mazekit.js', 'js/maze/maze-page.js', 'js/tutorial-demos.js', 'js/tutorial-sim.js', 'js/tour.js', 'js/measure-media.js', 'js/corridor-page.js', 'js/playground.js', 'js/train.js', 'js/measure.js',
  'firmware/robot_tuner_min.ino', 'fonts/archivo-latin.woff2', 'tracks/c90_l_20.track', 'tracks/c90_r_20.track', 'tracks/w20_s.track', 'tracks/m_s90_20.track', 'tracks/w20_err_A.track', 'tracks/w20_gaps_A.track', 'tracks/w20_bumps.track', 'tracks/w20_bend_l.track', 'tracks/o_gaps_20.track', 'tracks/smooth_loop.track', 'tracks/tight_broken.track', 'fonts/archivo-latin-ext.woff2',
];
self.addEventListener('install', (e) => {
  e.waitUntil(caches.open(VERSION).then((c) => Promise.all(CORE.map((u) => c.add(u).catch(() => null)))).then(() => self.skipWaiting()));
});
self.addEventListener('activate', (e) => {
  e.waitUntil(caches.keys().then((ks) => Promise.all(ks.filter((k) => k !== VERSION).map((k) => caches.delete(k)))).then(() => self.clients.claim()));
});
self.addEventListener('fetch', (e) => {
  const req = e.request;
  if (req.method !== 'GET' || new URL(req.url).origin !== location.origin) return;
  e.respondWith(
    fetch(req).then((res) => {
      if (res.ok) { const copy = res.clone(); caches.open(VERSION).then((c) => c.put(req, copy)); }
      return res;
    }).catch(() => caches.match(req, { ignoreSearch: true }).then((r) => r || caches.match('index.html')))
  );
});
