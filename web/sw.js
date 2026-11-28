// sw.js - offline cache. Every file of the site is cached on first visit; afterwards the site works offline.
// Network first, ALWAYS asking the server (cache: 'no-cache'): GitHub Pages lets browsers keep files 10 minutes, and
// a page made of some new and some old files breaks (e.g. two menus). Offline: the saved copy. Bump VERSION on release.
const VERSION = 'rt-v36';
const CORE = [
  './', 'index.html', 'builder.html', 'measure.html', 'corridor.html', 'playground.html', 'train.html', 'troubleshoot.html',
  'tutorial.html', 'maze.html', 'numbers.html', 'css/style.css', 'img/icon.svg', 'i18n/en.json', 'i18n/he.json', 'i18n/ar.json',
  'wasm/rt_sim.mjs', 'wasm/rt_sim.wasm', 'js/i18n.js', 'js/common.js', 'js/store.js', 'js/engine.js', 'js/draw.js', 'js/plot.js',
  'js/corridor.js', 'js/tracklib.js', 'js/simview.js', 'js/presets.js', 'js/train-core.js', 'js/train-worker.js', 'js/export.js', 'js/serial.js',
  'js/measure-steps.js', 'js/firmware-gen.js', 'js/builder.js', 'js/maze/maze.js', 'js/maze/algorithms.js', 'js/maze/mazekit.js', 'js/maze/maze-page.js', 'js/tutorial-demos.js', 'js/tutorial-sim.js', 'js/tour.js', 'js/measure-media.js', 'js/corridor-page.js', 'js/playground.js', 'js/train.js', 'js/measure.js', 'js/robotfile.js', 'js/simgate.js', 'js/cmd-buttons.js', 'js/save-reminder.js', 'js/link-status.js', 'js/own-code.js', 'js/own-code-ui.js', 'js/numbers.js',
  'firmware/robot_tuner_min.ino', 'fonts/archivo-latin.woff2', 'tracks/c90_l_20.track', 'tracks/c90_r_20.track', 'tracks/w20_s.track', 'tracks/m_s90_20.track', 'tracks/w20_err_A.track', 'tracks/w20_gaps_A.track', 'tracks/w20_bumps.track', 'tracks/w20_bend_l.track', 'tracks/o_gaps_20.track', 'tracks/smooth_loop.track', 'tracks/tight_broken.track', 'fonts/archivo-latin-ext.woff2',
];
self.addEventListener('install', (e) => {
  e.waitUntil(caches.open(VERSION).then((c) => Promise.all(CORE.map((u) => c.add(new Request(u, { cache: 'reload' })).catch(() => null)))).then(() => self.skipWaiting()));
});
self.addEventListener('activate', (e) => {
  e.waitUntil(caches.keys().then((ks) => Promise.all(ks.filter((k) => k !== VERSION).map((k) => caches.delete(k)))).then(() => self.clients.claim()));
});
self.addEventListener('fetch', (e) => {
  const req = e.request;
  if (req.method !== 'GET' || new URL(req.url).origin !== location.origin) return;
  e.respondWith(
    fetch(new Request(req.url, { cache: 'no-cache', credentials: 'same-origin' })).then((res) => {
      if (res.ok) { const copy = res.clone(); caches.open(VERSION).then((c) => c.put(req, copy)); }
      return res;
    }).catch(() => caches.match(req, { ignoreSearch: true }).then((r) => r || caches.match('index.html')))
  );
});
