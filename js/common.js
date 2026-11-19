// common.js - page chrome shared by every page: header + navigation, language menu, scope note, footer,
// offline service worker. Each page calls `await page()` before its own code.
import { initI18n, t, LANGS, currentLang, setLang, applyI18n } from './i18n.js';

export const PAGES = [
  { id: 'index', href: 'index.html', key: 'nav.home' },
  { id: 'builder', href: 'builder.html', key: 'nav.builder' },
  { id: 'measure', href: 'measure.html', key: 'nav.measure' },
  { id: 'corridor', href: 'corridor.html', key: 'nav.corridor' },
  { id: 'playground', href: 'playground.html', key: 'nav.playground' },
  { id: 'train', href: 'train.html', key: 'nav.train' },
  { id: 'troubleshoot', href: 'troubleshoot.html', key: 'nav.troubleshoot' },
  { id: 'tutorial', href: 'tutorial.html', key: 'nav.tutorial' },
  { id: 'maze', href: 'maze.html', key: 'nav.maze' },
];

export async function page() {
  await initI18n();
  const id = document.body.dataset.page;
  const header = document.createElement('header');
  header.className = 'site-header';
  header.innerHTML = `
    <a class="brand" href="index.html"><span class="logo" aria-hidden="true"></span><span>Robot Tuner</span></a>
    <button class="nav-toggle" aria-expanded="false" aria-controls="site-nav">☰ <span>${t('nav.menu')}</span></button>
    <nav id="site-nav" class="site-nav">${PAGES.map((p) => `<a href="${p.href}"${p.id === id ? ' aria-current="page"' : ''}>${t(p.key)}</a>`).join('')}</nav>
    <label class="lang"><span class="sr-only">${t('nav.language')}</span>
      <select id="lang-select">${LANGS.map((l) => `<option value="${l.code}"${l.code === currentLang() ? ' selected' : ''}>${l.name}</option>`).join('')}</select>
    </label>`;
  document.body.prepend(header);
  header.querySelector('#lang-select').addEventListener('change', (e) => setLang(e.target.value));
  const tog = header.querySelector('.nav-toggle');
  tog.addEventListener('click', () => { const o = header.classList.toggle('open'); tog.setAttribute('aria-expanded', o); });

  const footer = document.createElement('footer');
  footer.className = 'site-footer';
  footer.innerHTML = `<p>${t('footer.scope')}</p><p class="muted">${t('footer.offline')} · <a href="https://github.com/Basel-Shokha/Robot-Tuner">GitHub</a></p>`;
  document.body.append(footer);

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
