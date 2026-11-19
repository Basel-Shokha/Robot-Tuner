// node tools/i18n_sync.mjs - makes web/i18n/he.json and ar.json carry exactly the keys of en.json.
// Existing translations are kept; new keys get "" (the site then shows English for them); keys no longer
// in en.json are removed. Translators: fill the empty strings, keep {placeholders} and <b>/<a> markup.
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
const dir = new URL('../web/i18n/', import.meta.url);
const en = JSON.parse(readFileSync(new URL('en.json', dir), 'utf8'));
for (const lang of ['he', 'ar']) {
  const f = new URL(`${lang}.json`, dir);
  const old = existsSync(f) ? JSON.parse(readFileSync(f, 'utf8')) : {};
  const out = { '_meta.language': lang === 'he' ? 'עברית' : 'العربية', '_meta.direction': 'rtl' };
  for (const k of Object.keys(en)) out[k] = typeof old[k] === 'string' ? old[k] : '';
  writeFileSync(f, JSON.stringify(out, null, 1) + '\n');
  const done = Object.keys(en).filter((k) => out[k]).length;
  console.log(`${lang}.json: ${done} / ${Object.keys(en).length} translated`);
}
