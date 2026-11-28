// cmd-buttons.js - every serial command as a button with a plain-English label, grouped, with a small number box per
// argument (prefilled with the default). Used by the Measure console and the "I have my own code" connect step.
// BUTTONS covers every command of firmware-gen.js PROTOCOL (tests/js/owncode.test.mjs checks it).
import { el, t } from './common.js';

// cmd: as in PROTOCOL; args: [i18n label key, unit, default ('' = must be typed), required]
export const BUTTONS = [
  { group: 'wheels', cmd: 'CAL DIR', id: 'dir' },
  { group: 'wheels', cmd: 'CAL TICKS', id: 'ticks' },
  { group: 'wheels', cmd: 'CAL END', id: 'end' },
  { group: 'wheels', cmd: 'CAL MAXSPEED', id: 'maxspeed' },
  { group: 'wheels', cmd: 'CAL VMIN', id: 'vmin' },
  { group: 'wheels', cmd: 'CAL FF', id: 'ff' },
  { group: 'wheels', cmd: 'CAL STEP', id: 'step', args: [['cmd.a.speed', 'm/s', '0.15', false]] },
  { group: 'wheels', cmd: 'CAL TIMED', id: 'timed', args: [['cmd.a.time', 'ms', '2000', false]] },
  { group: 'floor', cmd: 'CAL DRIVE', id: 'drive', args: [['cmd.a.dist', 'm', '1', false]] },
  { group: 'floor', cmd: 'CAL DIST', id: 'dist', args: [['cmd.a.real_dist', 'm', '', true]] },
  { group: 'floor', cmd: 'CAL SPIN', id: 'spin', args: [['cmd.a.turns', '', '5', false]] },
  { group: 'floor', cmd: 'CAL TURNS', id: 'turns', args: [['cmd.a.real_turns', '', '', true]] },
  { group: 'floor', cmd: 'RUN', id: 'run', args: [['cmd.a.secs', 's', '10', false]] },
  { group: 'floor', cmd: 'STOP', id: 'stop', cls: 'danger' },
  { group: 'sensors', cmd: 'CAL TOF', id: 'tof', args: [['cmd.a.readings', '', '100', false], ['cmd.a.true_mm', 'mm', '', false]] },
  { group: 'sensors', cmd: 'STATUS', id: 'status' },
  { group: 'sensors', cmd: 'LOAD SENSORS', id: 'motors_off' },
  { group: 'sensors', cmd: 'LOAD FINAL', id: 'motors_on' },
];

/** send(cmdText); connected() -> bool. Returns the element; call .refresh() after (dis)connecting. */
export function commandButtons(send, connected) {
  const btns = [];
  const box = el('div', { class: 'cmd-buttons' });
  for (const g of ['wheels', 'floor', 'sensors']) {
    const row = el('div', { class: 'cmd-grid' });
    for (const b of BUTTONS.filter((x) => x.group === g)) {
      const inputs = (b.args || []).map(([lab, unit, def, req]) => el('input', { type: 'number', step: 'any', value: def, placeholder: req ? t('cmd.needed') : t('cmd.optional'),
        'aria-label': t(lab) + (unit ? ` (${unit})` : ''), title: t(lab) + (unit ? ` (${unit})` : '') }));
      const go = el('button', { class: 'small ' + (b.cls || ''), text: t('cmd.' + b.id), title: `${b.cmd}: ${t('cmd.' + b.id + '_h')}`, onclick: () => {
        const vals = inputs.map((i) => i.value.trim());
        const missing = (b.args || []).findIndex((a, k) => a[3] && !vals[k]);
        if (missing >= 0) { inputs[missing].focus(); inputs[missing].classList.add('flash'); return; }
        while (vals.length && !vals[vals.length - 1]) vals.pop();
        vals.forEach((v, k) => { if (!v) vals[k] = b.args[k][2]; });   // an empty box before a filled one: its default
        send([b.cmd, ...vals].join(' '));
      } });
      btns.push(go);
      row.append(el('div', { class: 'cmd-item' }, go, ...inputs.map((i, k) => el('label', { class: 'cmd-arg' }, i,
        el('span', { class: 'muted', text: `${t(b.args[k][0])}${b.args[k][1] ? ' (' + b.args[k][1] + ')' : ''}` }))),
        el('small', { class: 'muted', text: t('cmd.' + b.id + '_h') })));
    }
    box.append(el('h4', { text: t('cmd.g_' + g) }), row);
  }
  box.refresh = () => { for (const b of btns) b.disabled = !connected(); };
  box.refresh();
  return box;
}
