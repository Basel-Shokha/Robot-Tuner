// measure-media.js - what each Measure step shows: a real video when there is one, otherwise a short looping
// SVG/CSS animation of what to do with your hands (no external files, works offline).
//
// ┌──────────────────────────────────────────────────────────────────────────────────────────────────────┐
// │ VIDEO SLOTS. To replace an animation with a real video, put its address here:                         │
// │   a file next to the site ('img/measure/ticks.mp4' or .webm) plays inline, muted and looping;         │
// │   any other link (YouTube, Drive, ...) is shown as a "Watch the video" button next to the animation.  │
// │ Leave '' to keep the animation. Remember to bump VERSION in sw.js when you add files.                 │
// └──────────────────────────────────────────────────────────────────────────────────────────────────────┘
export const VIDEOS = {
  physical: '',
  ticks: '',
  maxspeed: '',
  vmin: '',
  ff: '',
  step: '',
  drive: '',
  spin: '',
  timed: '',
  tof: '',
  timing: '',
};

// colours come from the page's CSS variables, so the animations follow light / dark mode
const S = (w, h, body, label) => `<svg viewBox="0 0 ${w} ${h}" role="img" aria-label="${label}" class="anim">${body}</svg>`;

/** one wheel, a pen mark, a hand arrow and a counter "turn k of n" (the wheel turns n times per loop) */
function ticksAnim(n, label, turnWord) {
  const per = 1.6, T = n * per;
  let digits = '';
  for (let k = 0; k < n; k++) {
    const delay = -(T - k * per) % T;
    digits += `<text x="230" y="78" class="a-count" style="opacity:${k ? 0 : 1};animation:a-show ${T}s steps(1,end) ${delay}s infinite;--show:${100 / n}%">${turnWord} ${k + 1} / ${n}</text>`;
  }
  return S(320, 140, `
    <style>@keyframes a-show{0%{opacity:1}${(100 / n).toFixed(3)}%{opacity:0}100%{opacity:0}}</style>
    <line x1="10" y1="122" x2="190" y2="122" class="a-floor"/>
    <g style="transform-origin:100px 70px;animation:a-spin ${per}s linear infinite">
      <circle cx="100" cy="70" r="46" class="a-tyre"/><circle cx="100" cy="70" r="30" class="a-rim"/>
      <circle cx="100" cy="70" r="5" class="a-hub"/>
      <line x1="100" y1="24" x2="100" y2="40" class="a-mark"/>
    </g>
    <line x1="100" y1="6" x2="100" y2="20" class="a-ref"/><text x="108" y="16" class="a-small">start mark</text>
    <path d="M150 30 A60 60 0 0 1 160 100" class="a-arrow" marker-end="url(#ah)"/>
    <defs><marker id="ah" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" class="a-head"/></marker></defs>
    ${digits}
    <text x="230" y="100" class="a-small">by hand, slowly</text>`, label);
}

/** robot seen from above spinning in place over a floor mark, with a counter */
function spinAnim(n, label, turnWord) {
  const per = 1.4, T = n * per;
  let digits = '';
  for (let k = 0; k < n; k++) {
    const delay = -(T - k * per) % T;
    digits += `<text x="215" y="74" class="a-count" style="opacity:${k ? 0 : 1};animation:a-show2 ${T}s steps(1,end) ${delay}s infinite">${turnWord} ${k + 1} / ${n}</text>`;
  }
  return S(320, 140, `
    <style>@keyframes a-show2{0%{opacity:1}${(100 / n).toFixed(3)}%{opacity:0}100%{opacity:0}}</style>
    <line x1="100" y1="8" x2="100" y2="28" class="a-ref"/><text x="108" y="18" class="a-small">tape mark on the floor</text>
    <g style="transform-origin:100px 75px;animation:a-spin ${per}s linear infinite">
      <rect x="72" y="47" width="56" height="56" rx="8" class="a-body"/>
      <rect x="64" y="60" width="8" height="30" rx="2" class="a-tyre-f"/><rect x="128" y="60" width="8" height="30" rx="2" class="a-tyre-f"/>
      <path d="M100 40 L108 52 L92 52 Z" class="a-nose"/>
    </g>
    ${digits}
    <text x="215" y="96" class="a-small">count the nose passing the mark</text>`, label);
}

function liftedAnim(label) {
  return S(320, 140, `
    <rect x="95" y="92" width="130" height="34" rx="3" class="a-block"/><text x="160" y="114" class="a-small" text-anchor="middle">box / book</text>
    <rect x="105" y="40" width="110" height="34" rx="6" class="a-body"/>
    <g style="transform-origin:120px 80px;animation:a-spin .5s linear infinite"><circle cx="120" cy="80" r="14" class="a-tyre"/><line x1="120" y1="66" x2="120" y2="72" class="a-mark"/></g>
    <g style="transform-origin:200px 80px;animation:a-spin .5s linear infinite"><circle cx="200" cy="80" r="14" class="a-tyre"/><line x1="200" y1="66" x2="200" y2="72" class="a-mark"/></g>
    <text x="160" y="28" class="a-small" text-anchor="middle">wheels in the air</text>`, label);
}

function driveAnim(label, dist) {
  return S(320, 140, `
    <line x1="10" y1="110" x2="310" y2="110" class="a-floor"/>
    <line x1="30" y1="100" x2="30" y2="120" class="a-ref"/><line x1="270" y1="100" x2="270" y2="120" class="a-ref" style="animation:a-fade 4s linear infinite"/>
    <g class="a-tape"><line x1="30" y1="126" x2="270" y2="126"/><text x="150" y="138" class="a-small" text-anchor="middle">${dist}</text></g>
    <g style="animation:a-drive 4s ease-in-out infinite">
      <rect x="10" y="74" width="44" height="24" rx="5" class="a-body"/><circle cx="20" cy="100" r="8" class="a-tyre"/><circle cx="44" cy="100" r="8" class="a-tyre"/>
    </g>`, label);
}

function tofAnim(label) {
  return S(320, 140, `
    <rect x="40" y="50" width="60" height="40" rx="6" class="a-body"/><rect x="100" y="62" width="8" height="16" class="a-nose"/>
    <rect x="250" y="20" width="10" height="100" class="a-block"/><text x="255" y="134" class="a-small" text-anchor="middle">flat board</text>
    <line x1="110" y1="70" x2="248" y2="70" class="a-beam"/>
    <g class="a-tape"><line x1="108" y1="104" x2="250" y2="104"/><text x="180" y="98" class="a-small" text-anchor="middle">200 mm</text></g>`, label);
}

function rulerAnim(label) {
  return S(320, 140, `
    <rect x="110" y="30" width="100" height="80" rx="8" class="a-body"/>
    <rect x="98" y="50" width="12" height="40" rx="2" class="a-tyre-f"/><rect x="210" y="50" width="12" height="40" rx="2" class="a-tyre-f"/>
    <g style="animation:a-ruler 3s ease-in-out infinite"><rect x="104" y="114" width="112" height="14" class="a-rulerbar"/>
      ${Array.from({ length: 12 }, (_, i) => `<line x1="${106 + i * 10}" y1="114" x2="${106 + i * 10}" y2="${i % 5 ? 119 : 123}" class="a-ref"/>`).join('')}</g>
    <text x="160" y="20" class="a-small" text-anchor="middle">wheel centre to wheel centre, in mm</text>`, label);
}

/** the media element for one step: a video if one is set in VIDEOS, else the animation (null if none fits) */
export function stepMedia(id, { n, label, turnWord, videoWord, soonWord }) {
  const url = VIDEOS[id] || '';
  const box = document.createElement('figure');
  box.className = 'media';
  box.dataset.videoSlot = id;      // VIDEO SLOT: see VIDEOS at the top of this file
  if (/\.(mp4|webm)$/i.test(url)) {
    box.innerHTML = `<video src="${url}" autoplay loop muted playsinline controls aria-label="${label}"></video>`;
    return box;
  }
  const svg = {
    physical: () => rulerAnim(label), ticks: () => ticksAnim(n, label, turnWord), spin: () => spinAnim(n, label, turnWord),
    maxspeed: () => liftedAnim(label), vmin: () => liftedAnim(label), ff: () => liftedAnim(label), step: () => liftedAnim(label),
    drive: () => driveAnim(label, '1 m by the encoders → measure the real distance'), timed: () => driveAnim(label, '2 s at full power → measure'),
    tof: () => tofAnim(label),
  }[id];
  if (!svg) return null;
  box.innerHTML = svg() + `<figcaption>${url ? `<a class="btn small" href="${url}" target="_blank" rel="noopener">${videoWord}</a>` : `<span class="muted">${soonWord}</span>`}</figcaption>`;
  return box;
}
