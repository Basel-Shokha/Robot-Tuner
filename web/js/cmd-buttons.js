// cmd-buttons.js - every serial command as a button with a plain-English label, grouped, with a small number box per
// argument (prefilled with the default). Used by the Measure console and the "I have my own code" connect step.
// BUTTONS covers every command of firmware-gen.js PROTOCOL (tests/js/owncode.test.mjs checks it).
import { el, t } from './common.js';
import { PROTOCOL } from './firmware-gen.js';
import { stepMedia } from './measure-media.js';

// the animation each command's popup shows (measure-media.js ids)
const MEDIA = { dir: 'maxspeed', ticks: 'ticks', maxspeed: 'maxspeed', vmin: 'vmin', ff: 'ff', step: 'step', timed: 'timed', drive: 'drive', dist: 'drive',
  spin: 'spin', turns: 'spin', run: 'drive', tof: 'tof' };
export const motionOf = (cmd) => (PROTOCOL.find((p) => p.cmd === cmd) || {}).motion || 'still';
export function motionBadge(m) { return el('span', { class: 'motion m-' + m, title: t('cmd.m_' + m + '_h'), text: t('cmd.m_' + m) }); }

/** "Before you press Send": what to do on the robot, an animation, what happens next. Resolves true = send. */
function confirmDialog(b, cmdText) {
  return new Promise((res) => {
    const m = motionOf(b.cmd);
    const media = MEDIA[b.id] ? stepMedia(MEDIA[b.id], { n: 1, label: t('cmd.' + b.id), turnWord: t(b.id === 'spin' || b.id === 'turns' ? 'ms.rep.spin_word' : 'ms.rep.turn_word'), videoWord: t('ms.video') }) : null;
    const dlg = el('dialog', { class: 'cmd-dialog' });
    const close = (v) => { dlg.close(); dlg.remove(); res(v); };
    dlg.append(
      el('div', { class: 'row' }, motionBadge(m), el('span', { class: 'spacer' }), el('button', { class: 'small', text: '✕', 'aria-label': t('cmd.cancel'), onclick: () => close(false) })),
      el('h3', { text: t('cmd.' + b.id) }),
      el('p', { class: 'muted', text: t('cmd.m_' + m + '_h') }),
      media,
      el('h4', { text: t('cmd.do_h') }),
      el('ol', {}, ...t('cmd.' + b.id + '_do').split('|').map((x) => el('li', { text: x }))),
      el('p', {}, el('b', { text: t('cmd.next_h') + ' ' }), t('cmd.' + b.id + '_next')),
      el('div', { class: 'row' }, el('button', { class: 'primary ' + (b.cls || ''), text: t('cmd.send', { cmd: cmdText }), onclick: () => close(true) }),
        el('button', { text: t('cmd.cancel'), onclick: () => close(false) })));
    dlg.addEventListener('cancel', (e) => { e.preventDefault(); close(false); });
    document.body.append(dlg); dlg.showModal();
  });
}

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
  { group: 'sensors', cmd: 'PING', id: 'ping' },
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
      const go = el('button', { class: 'small ' + (b.cls || ''), text: t('cmd.' + b.id), title: `${b.cmd}: ${t('cmd.' + b.id + '_h')}`, onclick: async () => {
        const vals = inputs.map((i) => i.value.trim());
        const missing = (b.args || []).findIndex((a, k) => a[3] && !vals[k]);
        if (missing >= 0) { inputs[missing].focus(); inputs[missing].classList.add('flash'); return; }
        while (vals.length && !vals[vals.length - 1]) vals.pop();
        vals.forEach((v, k) => { if (!v) vals[k] = b.args[k][2]; });   // an empty box before a filled one: its default
        const text = [b.cmd, ...vals].join(' ');
        if (b.id === 'stop' || await confirmDialog(b, text)) send(text);   // STOP goes out at once
      } });
      btns.push(go);
      row.append(el('div', { class: 'cmd-item' }, motionBadge(motionOf(b.cmd)), go, ...inputs.map((i, k) => el('label', { class: 'cmd-arg' }, i,
        el('span', { class: 'muted', text: `${t(b.args[k][0])}${b.args[k][1] ? ' (' + b.args[k][1] + ')' : ''}` }))),
        el('small', { class: 'muted', text: t('cmd.' + b.id + '_h') })));
    }
    box.append(el('h4', { text: t('cmd.g_' + g) }), row);
    if (g === 'floor') row.before(el('p', { class: 'muted small-note', text: t('cmd.floor_usb') }));
  }
  box.refresh = () => { for (const b of btns) b.disabled = !connected(); };
  box.refresh();
  return box;
}
