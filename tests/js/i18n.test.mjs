// Every i18n key used by the site exists in en.json, and he/ar carry exactly the same keys.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, readdirSync } from 'node:fs';
const web = new URL('../../web/', import.meta.url);
const en = JSON.parse(readFileSync(new URL('i18n/en.json', web), 'utf8'));

function files(dir) {
  const out = [];
  for (const e of readdirSync(dir, { withFileTypes: true })) {
    const p = new URL(e.name + (e.isDirectory() ? '/' : ''), dir);
    if (e.isDirectory()) { if (!['wasm', 'firmware', 'i18n'].includes(e.name)) out.push(...files(p)); } else if (/\.(html|js)$/.test(e.name) && e.name !== 'i18n.js') out.push(p);
  }
  return out;
}

test('all used keys exist in en.json', () => {
  const missing = new Set();
  for (const f of files(web)) {
    const s = readFileSync(f, 'utf8');
    for (const m of s.matchAll(/data-i18n(?:-html)?="([^"]+)"/g)) if (!(m[1] in en)) missing.add(m[1]);
    for (const m of s.matchAll(/data-i18n-attr="([^"]+)"/g)) for (const pair of m[1].split(';')) { const k = pair.split(':')[1]; if (k && !(k.trim() in en)) missing.add(k.trim()); }
    for (const m of s.matchAll(/\bt\('([a-z][\w.]*)'\s*[,)]/g)) if (!(m[1] in en)) missing.add(m[1]);
    for (const m of s.matchAll(/key: '([a-z][\w.]*)'\s*[,}]/g)) if (!(m[1] in en)) missing.add(m[1]);
  }
  assert.deepEqual([...missing], []);
});

test('he.json and ar.json have exactly the keys of en.json (run tools/i18n_sync.mjs)', () => {
  for (const lang of ['he', 'ar']) {
    const d = JSON.parse(readFileSync(new URL(`i18n/${lang}.json`, web), 'utf8'));
    const keys = Object.keys(d).filter((k) => !k.startsWith('_meta.'));
    assert.deepEqual(keys.sort(), Object.keys(en).sort(), lang);
  }
});
