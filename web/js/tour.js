// tour.js - the "Guide" button: a step-by-step overlay that highlights one element at a time and says what it
// does. Steps per page are in TOURS (selector + i18n key tour.<page>.<name>); elements that are missing or hidden
// (closed tabs, narrow screens) are skipped. Whether a page's tour was seen is remembered in localStorage; on the
// first visit of a page the tour is offered (not forced).
import { t } from './i18n.js';

export const TOURS = {
  index: [['.page-head', 'what'], ['#start-here', 'start'], ['#route', 'route'], ['.extras', 'extras'], ['#knowhow', 'know'], ['.guide-btn', 'guide']],
  builder: [['.page-head', 'what'], ['#robot-name', 'name'], ['#drive', 'drive'], ['.tabs', 'tabs'], ['.canvas-wrap', 'canvas'], ['#checks', 'checks'], ['[data-tab="file"]', 'file'], ['.next-step', 'next']],
  measure: [['.page-head', 'what'], ['#connect', 'connect'], ['#manual', 'manual'], ['#steps .step', 'physical'], ['#steps .step:nth-child(2)', 'step'], ['#log', 'console'], ['.next-step', 'next']],
  corridor: [['.page-head', 'what'], ['.tabs', 'mode'], ['#env-errors', 'errors'], ['#env-start', 'start'], ['#run', 'run'], ['.canvas-wrap', 'canvas'], ['#plots', 'plots'], ['.next-step', 'next']],
  train: [['.page-head', 'what'], ['input[name="tset"]', 'setup'], ['#speed-frac', 'speed'], ['#effort', 'effort'], ['#start', 'start'], ['#plots', 'plots'], ['.next-step', 'next']],
  playground: [['.page-head', 'what'], ['#gains', 'gains'], ['#presets', 'presets'], ['#sweep', 'sweep'], ['#run', 'run'], ['#save-gains', 'save'], ['#plots', 'plots']],
  tutorial: [['.page-head', 'what'], ['#toc', 'toc'], ['#demo-d', 'demo'], ['#demo-graph', 'graph'], ['.deeper', 'deeper']],
  troubleshoot: [['.page-head', 'what'], ['#list', 'list']],
  maze: [['.page-head', 'what'], ['#maze-set', 'maze'], ['#algo-set', 'algo'], ['#run', 'run'], ['#cv-map', 'map']],
};

const KEY = (page) => 'rt.tour.' + page;
export function tourSeen(page) { try { return localStorage.getItem(KEY(page)) === '1'; } catch (e) { return false; } }
export function markSeen(page) { try { localStorage.setItem(KEY(page), '1'); } catch (e) { /* storage blocked: offer again next time */ } }

function visible(e) {
  if (!e) return false;
  const r = e.getBoundingClientRect();
  return r.width > 0 && r.height > 0 && getComputedStyle(e).visibility !== 'hidden';
}
/** the steps of a page whose element exists and is visible now (a radio/checkbox is highlighted with its label) */
export function resolveSteps(page, root = document) {
  return (TOURS[page] || []).map(([sel, name]) => {
    let e = root.querySelector(sel);
    if (e && e.matches('input[type=radio], input[type=checkbox]')) e = e.closest('label') || e;
    return { e, name };
  }).filter((s) => visible(s.e));
}

let active = null;
export function startTour(page) {
  if (active) active.close();
  document.querySelector('.tour-offer')?.remove();
  const steps = resolveSteps(page);
  if (!steps.length) return;
  let i = 0;
  const hole = document.createElement('div'); hole.className = 'tour-hole';
  const box = document.createElement('div'); box.className = 'tour-box'; box.setAttribute('role', 'dialog'); box.setAttribute('aria-modal', 'false');
  box.setAttribute('aria-labelledby', 'tour-title');
  const shield = document.createElement('div'); shield.className = 'tour-shield';
  document.body.append(shield, hole, box);
  const place = () => {
    if (!steps[i] || !box.isConnected) return;
    const r = steps[i].e.getBoundingClientRect(), pad = 6;
    Object.assign(hole.style, { top: r.top - pad + 'px', left: r.left - pad + 'px', width: r.width + 2 * pad + 'px', height: r.height + 2 * pad + 'px' });
    const bw = Math.min(340, innerWidth - 24), bh = box.offsetHeight;
    let top = r.bottom + 14;
    if (top + bh > innerHeight - 8) top = r.top - bh - 14;           // no room below: above
    if (top < 8) top = Math.max(8, Math.min(innerHeight - bh - 8, r.top + 12));  // tall element: inside it
    const left = Math.max(12, Math.min(innerWidth - bw - 12, r.left));
    Object.assign(box.style, { top: top + 'px', left: left + 'px', width: bw + 'px' });
  };
  const show = () => {
    const s = steps[i], last = i === steps.length - 1;
    box.innerHTML = `<div class="tour-count">${t('tour.count', { n: i + 1, total: steps.length })}</div>
      <h3 id="tour-title">${t(`tour.${page}.${s.name}.h`)}</h3><p>${t(`tour.${page}.${s.name}`)}</p>
      <div class="tour-btns"><button type="button" class="small" data-a="skip">${t(last ? 'tour.close' : 'tour.skip')}</button><span class="spacer"></span>
      ${i ? `<button type="button" class="small" data-a="back">${t('tour.back')}</button>` : ''}
      <button type="button" class="small primary" data-a="next">${t(last ? 'tour.done' : 'tour.next')}</button></div>`;
    s.e.scrollIntoView({ block: 'center', behavior: matchMedia('(prefers-reduced-motion: reduce)').matches ? 'auto' : 'smooth' });
    place(); setTimeout(place, 350);
    box.querySelector('[data-a=next]').focus({ preventScroll: true });
  };
  const close = () => { markSeen(page); hole.remove(); box.remove(); shield.remove(); removeEventListener('resize', place); removeEventListener('scroll', place, true); removeEventListener('keydown', key); active = null; };
  const go = (d) => { i += d; if (i < 0) i = 0; if (i >= steps.length) close(); else show(); };
  const key = (ev) => { if (ev.key === 'Escape') close(); else if (ev.key === 'ArrowRight') go(1); else if (ev.key === 'ArrowLeft') go(-1); };
  box.addEventListener('click', (ev) => { const a = ev.target.closest('[data-a]')?.dataset.a; if (a === 'next') go(1); else if (a === 'back') go(-1); else if (a === 'skip') close(); });
  shield.addEventListener('click', close);
  addEventListener('resize', place); addEventListener('scroll', place, true); addEventListener('keydown', key);
  active = { close };
  show();
}

/** first visit of a page: a small offer in the corner, not a forced tour */
export function offerTour(page) {
  if (tourSeen(page) || !TOURS[page]) return;
  const n = document.createElement('div'); n.className = 'tour-offer'; n.setAttribute('role', 'status');
  n.innerHTML = `<p>${t('tour.offer')}</p><div class="tour-btns"><button type="button" class="small" data-a="no">${t('tour.no')}</button><button type="button" class="small primary" data-a="yes">${t('tour.yes')}</button></div>`;
  n.addEventListener('click', (ev) => {
    const a = ev.target.closest('[data-a]')?.dataset.a; if (!a) return;
    n.remove(); markSeen(page); if (a === 'yes') startTour(page);
  });
  document.body.append(n);
}
