// link-status.js - "is the robot really there?": while the port is open, PING every 3 s (hidden from the log) and show
// whether anything came back. Any received line counts; PONG proves the robot understands Robot Tuner's commands.
import { el, t, toast } from './common.js';
import { usesBt, btName, normalizeHardware } from './firmware-gen.js';

/** "No cable: Bluetooth" how-to for ESP32 robots with Bluetooth on (null otherwise) */
export function btHelp(robot) {
  const hw = normalizeHardware(robot.hardware, robot.sensors.length);
  if (!usesBt(hw)) return null;
  const name = btName(robot, hw);
  return el('details', { class: 'card bt-help' }, el('summary', { text: t('bt.h') }),
    el('ol', {}, ...t('bt.steps', { name }).split('|').map((x) => el('li', { text: x }))),
    el('p', { class: 'muted small-note', text: t('bt.no_wifi') }));
}

/** returns a pill element that updates itself; state: off | waiting | ok | silent | foreign */
export function linkStatus(link) {
  const pill = el('span', { class: 'link-status off', role: 'status', 'aria-live': 'polite' });
  let last = '';
  link.onLink && link.onLink((s) => toast(t('link.bt_' + s), s === 'connected' ? 'ok' : 'warn'));
  const tick = () => {
    const now = Date.now();
    let st;
    if (!link.connected) st = link.lost ? 'lost' : 'off';
    else {
      if (now - (link.lastPingAt || 0) > 3000) { link.lastPingAt = now; link.ping(); }
      if (link.lastPong && now - link.lastPong < 7000) st = 'ok';
      else if (link.lastRx && now - link.lastRx < 7000) st = 'foreign';   // talks, but no PONG: old sketch or other code
      else st = now - link.openedAt < 7000 ? 'waiting' : 'silent';
    }
    let txt = t('link.' + st, { s: link.lastPong ? Math.round((now - link.lastPong) / 1000) : 0 });
    if (st === 'ok' && link.via) txt = t('link.ok_via', { via: t('link.via_' + (link.via === 'bluetooth' ? 'bt' : 'usb')) });
    if (st + txt !== last) { last = st + txt; pill.className = 'link-status ' + st; pill.textContent = txt; pill.title = t('link.' + st + '_h'); }
  };
  tick(); setInterval(tick, 1000);
  return pill;
}
