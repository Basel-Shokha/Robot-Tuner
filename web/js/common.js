// common.js - page chrome shared by every page: header + navigation, language menu, scope note, footer,
// offline service worker. Each page calls `await page()` before its own code.
import { initI18n, t, applyI18n } from './i18n.js';
import { startTour, offerTour } from './tour.js';

// The workflow is a real sequence (each step feeds the next), so it is numbered; the rest is not.
export const STEPS = [
  { id: 'builder', href: 'builder.html', key: 'nav.builder' },
  { id: 'measure', href: 'measure.html', key: 'nav.measure' },
  { id: 'corridor', href: 'corridor.html', key: 'nav.corridor' },
  { id: 'train', href: 'train.html', key: 'nav.train' },
  { id: 'playground', href: 'playground.html', key: 'nav.playground' },
];
export const LEARN = [
  { id: 'tutorial', href: 'tutorial.html', key: 'nav.tutorial' },
  { id: 'troubleshoot', href: 'troubleshoot.html', key: 'nav.troubleshoot' },
  { id: 'numbers', href: 'numbers.html', key: 'nav.numbers' },
];
export const BEYOND = [{ id: 'maze', href: 'maze.html', key: 'nav.maze' }];
export const PAGES = [{ id: 'index', href: 'index.html', key: 'nav.home' }, ...STEPS, ...LEARN, ...BEYOND];

const LOGO = '<svg width="26" height="26" viewBox="0 0 26 26" aria-hidden="true"><rect x="1" y="2" width="2.5" height="22" rx="1" fill="currentColor"/>'
  + '<rect x="22.5" y="2" width="2.5" height="22" rx="1" fill="currentColor"/><rect x="8.5" y="7" width="9" height="12" rx="2" fill="#e8a10c"/></svg>';

function visited(id) {
  try {
    const v = new Set(JSON.parse(localStorage.getItem('rt.visited') || '[]'));
    if (id) { v.add(id); localStorage.setItem('rt.visited', JSON.stringify([...v])); }
    return v;
  } catch (e) { return new Set(); }
}

function theme() { try { return localStorage.getItem('rt.theme') === 'dark' ? 'dark' : 'light'; } catch (e) { return 'light'; } }
function applyTheme(th) {
  document.documentElement.dataset.theme = th;
  try { localStorage.setItem('rt.theme', th); } catch (e) { /* ignore */ }
  document.querySelectorAll('.theme-toggle').forEach((b) => { b.textContent = t(th === 'dark' ? 'nav.theme_light' : 'nav.theme_dark'); });
  window.dispatchEvent(new Event('resize'));  // canvases redraw with the new colours
}

export async function page() {
  document.documentElement.dataset.theme = theme();
  await initI18n();
  const id = document.body.dataset.page;
  const seen = visited(id);
  const main = document.querySelector('main');
  const app = document.createElement('div');
  app.className = 'app';
  // the menu: three titles; the arrow opens the list underneath (the group of the current page starts open)
  const group = (g, key, pages, body) => `<details class="rail-group" data-group="${g}"${pages.some((p) => p.id === id) ? ' open' : ''}>
        <summary class="rail-title">${t(key)}</summary>${body}</details>`;
  const link = (p) => `<a href="${p.href}"${p.id === id ? ' aria-current="page"' : ''}>${t(p.key)}</a>`;
  app.innerHTML = `
    <aside class="rail" id="rail" aria-label="${t('nav.menu')}">
      <a class="brand" href="index.html">${LOGO}<span>Robot Tuner</span></a>
      ${group('steps', 'nav.group_steps', STEPS, `<ol class="track">${STEPS.map((p, i) => `<li class="${p.id === id ? 'current' : seen.has(p.id) ? 'visited' : ''}"><a href="${p.href}"${p.id === id ? ' aria-current="page"' : ''}><span class="stop"><span>${i + 1}</span></span>${t(p.key)}</a></li>`).join('')}</ol>`)}
      ${group('learn', 'nav.group_learn', LEARN, `<nav class="rail-links">${LEARN.map(link).join('')}</nav>`)}
      ${group('beyond', 'nav.group_beyond', BEYOND, `<nav class="rail-links">${BEYOND.map(link).join('')}</nav>`)}
      <div class="rail-foot">
        <button class="small theme-toggle" type="button"></button>
        <span>${t('footer.offline')}</span>
      </div>
    </aside>
    <div class="page">
      <div class="topbar"><button class="small" id="nav-toggle" aria-controls="rail" aria-expanded="false">${t('nav.menu')}</button>
        <a class="brand" href="index.html">${LOGO}<span>Robot Tuner</span></a><button class="small theme-toggle" type="button"></button></div>
    </div>`;
  main.replaceWith(app);
  const pageEl = app.querySelector('.page');
  pageEl.append(main);
  const footer = document.createElement('footer');
  footer.className = 'site-footer';
  footer.innerHTML = `<p>${t('footer.scope')}</p><p class="credits">${t('footer.by')}</p>`;
  pageEl.append(footer);

  // every page: a head row with the Guide button; step pages also get "Step n of 5" above the title and the way on
  const si = STEPS.findIndex((p) => p.id === id);
  const h1 = main.querySelector('h1');
  if (h1) {
    const line = document.createElement('div');
    line.className = 'stepline';
    line.innerHTML = (si >= 0 ? `<b>${t('page.step', { n: si + 1, total: STEPS.length })}</b>` : '<span></span>')
      + `<button type="button" class="small guide-btn" aria-label="${t('tour.btn_label')}"><span aria-hidden="true">?</span> ${t('tour.btn')}</button>`;
    const head = document.createElement('div');
    head.className = 'page-head';
    h1.before(head);
    head.append(line, h1);
    const intro = head.nextElementSibling;
    if (intro && intro.tagName === 'P') head.append(intro);
    line.querySelector('.guide-btn').addEventListener('click', () => startTour(id));
    setTimeout(() => offerTour(id), 900);
  }
  if (si >= 0 && si + 1 < STEPS.length) {
    const nx = STEPS[si + 1];
    const box = document.createElement('div');
    box.className = 'next-step';
    box.innerHTML = `<a class="btn primary" href="${nx.href}">${t('page.next', { name: t(nx.key) })}</a>`;
    main.append(box);
  }

  app.querySelectorAll('.theme-toggle').forEach((b) => b.addEventListener('click', () => applyTheme(theme() === 'dark' ? 'light' : 'dark')));
  applyTheme(theme());
  const tog = app.querySelector('#nav-toggle');
  tog.addEventListener('click', () => { const o = app.classList.toggle('nav-open'); tog.setAttribute('aria-expanded', o); });
  main.addEventListener('click', () => { app.classList.remove('nav-open'); tog.setAttribute('aria-expanded', 'false'); });

  if ('serviceWorker' in navigator && location.protocol !== 'file:') {
    navigator.serviceWorker.register('sw.js').catch(() => { /* offline cache is optional */ });
  }
}

// Small DOM helpers ----------------------------------------------------------------------------------
export const $ = (sel, root = document) => root.querySelector(sel);
export const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));
export function el(tag, attrs = {}, ...kids) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') e.className = v;
    else if (k === 'text') e.textContent = v;
    else if (k === 'html') e.innerHTML = v;
    else if (k.startsWith('on')) e.addEventListener(k.slice(2), v);
    else if (v !== undefined && v !== null && v !== false) e.setAttribute(k, v === true ? '' : v);
  }
  for (const k of kids) if (k !== null && k !== undefined) e.append(k);
  return e;
}

/** A labelled range slider with a number box.
 * opts: {key (i18n label) | label, min, max, step, value, unit, onInput, hint, log (logarithmic, 0 at the far left)} */
export function slider(opts) {
  const id = 'sl-' + Math.random().toString(36).slice(2, 8);
  const wrap = el('div', { class: 'slider' });
  const lab = el('label', { for: id }, el('span', { text: opts.label || t(opts.key) }));
  if (opts.unit) lab.append(el('span', { class: 'unit', text: ' (' + opts.unit + ')' }));
  const lo = opts.log ? Math.max(opts.min, opts.max * 1e-4) : opts.min;
  const toPos = (v) => (opts.log ? (v <= lo * 1.0001 ? 0 : Math.round(Math.log(v / lo) / Math.log(opts.max / lo) * 1000)) : v);
  const fromPos = (p) => (opts.log ? (+p === 0 ? 0 : +(lo * Math.pow(opts.max / lo, p / 1000)).toPrecision(3)) : +p);
  const range = el('input', { type: 'range', id, min: opts.log ? 0 : opts.min, max: opts.log ? 1000 : opts.max, step: opts.log ? 1 : opts.step, value: toPos(opts.value) });
  const num = el('input', { type: 'number', min: opts.min, step: opts.step || 'any', value: opts.value, 'aria-label': opts.label || t(opts.key) });
  const set = (v, fire = true) => { range.value = toPos(v); num.value = v; if (fire && opts.onInput) opts.onInput(+v); };
  range.addEventListener('input', () => set(fromPos(range.value)));
  num.addEventListener('change', () => { const v = parseFloat(num.value); if (Number.isFinite(v)) set(v); });
  wrap.append(lab, el('div', { class: 'slider-row' }, range, num));
  if (opts.hint) wrap.append(el('small', { class: 'muted', text: opts.hint }));
  wrap.set = set;
  wrap.get = () => +num.value;
  return wrap;
}

export function toggle(opts) {
  const id = 'tg-' + Math.random().toString(36).slice(2, 8);
  const cb = el('input', { type: 'checkbox', id });
  cb.checked = !!opts.value;
  cb.addEventListener('change', () => opts.onChange && opts.onChange(cb.checked));
  const w = el('label', { class: 'toggle', for: id }, cb, el('span', { text: opts.label || t(opts.key) }));
  w.input = cb;
  return w;
}

export function download(name, text, type = 'text/plain') {
  const a = el('a', { href: URL.createObjectURL(new Blob([text], { type })), download: name });
  document.body.append(a); a.click(); a.remove();
}
export function pickFile(accept) {
  return new Promise((res) => {
    const i = el('input', { type: 'file', accept });
    i.addEventListener('change', async () => { const f = i.files[0]; res(f ? { name: f.name, text: await f.text() } : null); });
    i.click();
  });
}
export function toast(msg, kind = 'info') {
  let box = $('#toasts');
  if (!box) { box = el('div', { id: 'toasts', 'aria-live': 'polite' }); document.body.append(box); }
  const n = el('div', { class: 'toast ' + kind, text: msg });
  box.append(n);
  setTimeout(() => n.remove(), 4000);
}
export { t, applyI18n };
