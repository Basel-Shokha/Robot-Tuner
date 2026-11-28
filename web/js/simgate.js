// simgate.js - before simulating: every REQUIRED robot value must be set (robotfile.js missingForSim). Otherwise the
// page shows what is missing, links each value to its field in the Robot builder, and the simulator stays locked.
import { el, t } from './common.js';
import { loadRobot } from './store.js';
import { missingForSim, fieldLink } from './robotfile.js';

/** call right after page(); returns true when the robot is ready to simulate */
export function robotReady() {
  const miss = missingForSim(loadRobot());
  if (!miss.length) return true;
  const main = document.querySelector('main'), head = main.querySelector('.page-head');
  for (const c of main.children) if (c !== head) { c.classList.add('gated'); c.setAttribute('inert', ''); }
  const card = el('section', { class: 'card gate', id: 'sim-gate' },
    el('h2', { text: t('gate.h') }), el('p', { text: t('gate.p', { n: miss.length }) }),
    el('a', { class: 'btn primary', href: fieldLink(miss[0]), text: t('gate.go', { name: miss[0].key }) }),
    el('ul', {}, ...miss.map((f) => el('li', {}, el('a', { href: fieldLink(f), text: f.key }), ` (${f.unit || f.section})${f.note ? ' - ' + f.note : ''}`))),
    el('p', { class: 'muted', text: t('gate.file') }));
  (head || main.firstChild).after(card);
  return false;
}
