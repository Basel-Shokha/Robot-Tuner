// own-code-ui.js - the "I have my own code" steps on the builder page (builder.html?own=1):
// 1. hardware (the builder form below), 2. COPY PROMPT + reference sketch, 3. upload in the Arduino IDE + CONNECT.
import { $, el, t, download, toast } from './common.js';
import { studentPrompt } from './own-code.js';
import { generateSketch } from './firmware-gen.js';
import { RobotLink, serialSupported } from './serial.js';
import { commandButtons } from './cmd-buttons.js';
import { linkStatus } from './link-status.js';

export const REF_NAME = 'robot_tuner_reference.ino';
export const isOwnMode = () => new URLSearchParams(location.search).get('own') === '1';

/** the two choices at the top of the builder page */
export function choiceBar(own) {
  const a = (href, key, note, on) => el('a', { class: 'choice' + (on ? ' on' : ''), href, 'aria-current': on ? 'true' : null },
    el('b', { text: t(key) }), el('small', { text: t(note) }));
  return el('div', { class: 'choices', id: 'code-choice' },
    a('builder.html', 'choice.gen', 'choice.gen_note', !own), a('builder.html?own=1', 'choice.own', 'choice.own_note', own));
}

async function copyText(txt, box) {
  try { await navigator.clipboard.writeText(txt); toast(t('own.copied')); return; } catch (e) { /* fall back below */ }
  try {
    const ta = el('textarea', { style: 'position:fixed;opacity:0' }); ta.value = txt; document.body.append(ta); ta.select();
    const ok = document.execCommand('copy'); ta.remove();
    if (ok) { toast(t('own.copied')); return; }
  } catch (e) { /* ignore */ }
  box.open = true; toast(t('own.copy_fail'));
}

/** builds the three steps into `root`; getRobot() returns the current robot; openTab(id) shows a builder tab */
export function mountOwn(root, getRobot, openTab) {
  const prompt = () => studentPrompt(getRobot(), getRobot().hardware, { sketchName: REF_NAME });
  const pre = el('pre', { class: 'code prompt-preview', id: 'own-preview' });
  const prev = el('details', { class: 'prompt-box' }, el('summary', { text: t('own.preview') }), pre);
  const refresh = () => { pre.textContent = prompt(); };

  const step = (n, hKey, ...body) => el('li', { class: 'own-step' }, el('span', { class: 'n', text: String(n) }),
    el('div', { class: 'stack' }, el('h3', { text: t(hKey) }), ...body));

  // 3: connect
  const link = new RobotLink();
  const log = el('pre', { class: 'code own-log', hidden: true });
  const state = el('span', { class: 'pill', text: t('ms.disconnected') });
  const cmd = el('input', { placeholder: t('own.cmd_ph'), style: 'flex:1;min-width:120px' });
  const send = el('button', { text: t('own.send'), disabled: true });
  const conn = el('button', { class: 'primary', id: 'own-connect', text: t('own.connect') });
  const lines = [];
  link.onLine((l) => {
    lines.push(l); if (lines.length > 200) lines.shift();
    log.hidden = false; log.textContent = lines.join('\n'); log.scrollTop = log.scrollHeight;
  });
  const cmdBox = commandButtons((c) => link.connected && link.send(c), () => link.connected);
  const upd = () => {
    cmdBox.refresh();
    state.textContent = t(link.connected ? 'ms.connected' : 'ms.disconnected'); state.className = 'pill ' + (link.connected ? 'ok' : '');
    conn.textContent = t(link.connected ? 'own.disconnect' : 'own.connect'); send.disabled = !link.connected;
  };
  conn.addEventListener('click', async () => {
    try { if (link.connected) await link.disconnect(); else { await link.connect(); setTimeout(() => link.connected && link.send('STATUS'), 1500); } } catch (e) { toast(String(e.message || e)); }
    upd();
  });
  const doSend = () => { const c = cmd.value.trim(); if (c && link.connected) link.send(c); cmd.value = ''; };
  send.addEventListener('click', doSend);
  cmd.addEventListener('keydown', (e) => { if (e.key === 'Enter') doSend(); });
  if (!serialSupported()) conn.disabled = true;

  root.append(el('div', { class: 'card own' },
    el('h2', { text: t('own.h') }),
    el('p', { class: 'muted', text: t('own.intro') }),
    el('ol', { class: 'own-steps' },
      step(1, 'own.s1_h', el('p', { html: t('own.s1') }),
        el('div', { class: 'row' }, el('button', { text: t('own.s1_btn'), onclick: () => { openTab('firmware'); $('.tabs').scrollIntoView({ behavior: 'smooth', block: 'start' }); } }))),
      step(2, 'own.s2_h', el('p', { text: t('own.s2') }),
        el('div', { class: 'row' },
          el('button', { class: 'primary', id: 'own-copy', text: t('own.copy'), onclick: () => copyText(prompt(), prev) }),
          el('button', { id: 'own-ref', text: t('own.ref'), onclick: () => { const r = getRobot(); download(REF_NAME, generateSketch(r, r.hardware).code); } }),
          el('button', { class: 'small', text: t('own.md'), onclick: () => download('robot_tuner_prompt.md', prompt(), 'text/markdown') })),
        el('small', { class: 'muted', text: t('own.prompt_note') }), prev),
      step(3, 'own.s3_h', el('p', { text: t('own.s3') }),
        el('p', { class: 'muted small-note', text: t('own.compile_note') }),
        el('div', { class: 'row' }, conn, state, linkStatus(link)),
        serialSupported() ? null : el('p', { class: 'muted', text: t('own.nosupport') }),
        cmdBox, el('p', { class: 'muted small-note', text: t('cmd.or_type') }), el('div', { class: 'row' }, cmd, send), log,
        el('p', { html: t('own.fail') }),
        el('p', { html: t('own.next') })))));
  refresh();
  window.addEventListener('robot-changed', refresh);
  return { refresh };
}
