// i18n.js - every user-visible string comes from web/i18n/<lang>.json.
//   HTML:  <span data-i18n="key"></span>, <input data-i18n-attr="placeholder:key;title:key2">,
//          <p data-i18n-html="key"> (trusted markup from our own json files, e.g. <b>, <a>)
//   JS:    t('key', {n: 3}) -> replaces {n}
// he / ar files hold the same keys; an empty value falls back to English. Hebrew and Arabic switch the page to
// right-to-left (<html dir="rtl">); the CSS uses logical properties (margin-inline-start ...), canvases stay LTR.
export const LANGS = [
  { code: 'en', name: 'English', dir: 'ltr' },
  { code: 'he', name: 'עברית', dir: 'rtl' },
  { code: 'ar', name: 'العربية', dir: 'rtl' },
];
let dict = {}, fallback = {}, lang = 'en';

function base() { return new URL('../', import.meta.url); }

export function currentLang() { return lang; }
export function getLangPref() {
  try { const l = localStorage.getItem('rt.lang'); if (LANGS.some((x) => x.code === l)) return l; } catch (e) { /* storage blocked */ }
  return 'en';
}

export async function initI18n(code = getLangPref()) {
  const load = async (c) => { try { const r = await fetch(new URL(`i18n/${c}.json`, base())); return r.ok ? r.json() : {}; } catch (e) { return {}; } };
  fallback = await load('en');
  dict = code === 'en' ? fallback : await load(code);
  lang = code;
  const L = LANGS.find((x) => x.code === code) || LANGS[0];
  document.documentElement.lang = code;
  document.documentElement.dir = L.dir;
  applyI18n(document);
}

export function setLang(code) {
  try { localStorage.setItem('rt.lang', code); } catch (e) { /* ignore */ }
  location.reload();
}

export function t(key, vars) {
  let s = dict[key];
  if (s === undefined || s === '') s = fallback[key];
  if (s === undefined) { console.warn('i18n: missing key', key); s = key; }
  if (vars) for (const [k, v] of Object.entries(vars)) s = s.split(`{${k}}`).join(String(v));
  return s;
}
export function has(key) { return fallback[key] !== undefined; }

export function applyI18n(root) {
  root.querySelectorAll('[data-i18n]').forEach((el) => { el.textContent = t(el.dataset.i18n); });
  root.querySelectorAll('[data-i18n-html]').forEach((el) => { el.innerHTML = t(el.dataset.i18nHtml); });
  root.querySelectorAll('[data-i18n-attr]').forEach((el) => {
    for (const pair of el.dataset.i18nAttr.split(';')) {
      const [attr, key] = pair.split(':').map((s) => s.trim());
      if (attr && key) el.setAttribute(attr, t(key));
    }
  });
  const title = document.querySelector('title[data-i18n]');
  if (title) document.title = t(title.dataset.i18n) + ' · Robot Tuner';
}

// Numbers: always Western digits with a dot (students copy them into code).
export function fmt(x, digits = 2) {
  if (x === null || x === undefined || Number.isNaN(x)) return '–';
  return Number(x).toFixed(digits);
}
