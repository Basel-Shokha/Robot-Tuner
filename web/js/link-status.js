// link-status.js - "is the robot really there?": while the port is open, PING every 3 s (hidden from the log) and show
// whether anything came back. Any received line counts; PONG proves the robot understands Robot Tuner's commands.
import { el, t, toast } from './common.js';
import { serialSupported, bleSupported } from './serial.js';
import { usesBt, btName, normalizeHardware, generateSketch } from './firmware-gen.js';

/** "No cable: Bluetooth" how-to for ESP32 robots with Bluetooth on (null otherwise) */
export function btHelp(robot) {
  const hw = normalizeHardware(robot.hardware, robot.sensors.length);
  if (!usesBt(hw)) return null;
  const name = btName(robot, hw);
  return el('details', { class: 'card bt-help' }, el('summary', { text: t('bt.h') }),
    el('ol', {}, ...t('bt.steps', { name }).split('|').map((x) => el('li', { text: x }))),
    el('p', { class: 'muted small-note', text: t('bt.no_wifi') }));
}

/** the two connect buttons (USB cable, Bluetooth): the one in use turns into Disconnect, the other waits.
 *  usb: an existing button to reuse (Measure keeps its #connect); after(): called after every change */
export function connectButtons(link, after = () => {}, usb = null) {
  usb = usb || el('button', { class: 'primary' });
  const ble = el('button', { class: 'primary', id: usb.id ? usb.id + '-ble' : null });
  const upd = () => {
    const m = link.mode;
    usb.textContent = t(m === 'usb' ? 'ms.disconnect' : 'ms.connect_usb');
    ble.textContent = t(m === 'ble' ? 'ms.disconnect' : 'ms.connect_ble');
    usb.disabled = !serialSupported() || m === 'ble';
    ble.disabled = !bleSupported() || m === 'usb';
    ble.title = bleSupported() ? '' : t('ms.no_ble');
    after();
  };
  const go = (kind) => async () => {
    try {
      if (link.connected) await link.disconnect();
      else if (kind === 'ble') await link.connectBle();
      else await link.connect();
    } catch (e) { if (e && e.name !== 'NotFoundError') toast(t('ms.conn_err') + ' ' + (e.message || e), 'bad'); }
    upd();
  };
  usb.addEventListener('click', go('usb'));
  ble.addEventListener('click', go('ble'));
  link.onLine((l) => { if (l.startsWith('# connection lost')) upd(); });
  upd();
  return { usb, ble, refresh: upd };
}

/** returns a pill element that updates itself; state: off | waiting | ok | silent | foreign */
/** getRobot (optional): the robot profile; then the status also says when the robot runs older code than these settings */
export function linkStatus(link, getRobot = null) {
  const pill = el('span', { class: 'link-status off', role: 'status', 'aria-live': 'polite' });
  const stale = el('span', { class: 'link-status silent', hidden: true, text: t('link.stale'), title: t('link.stale_h') });
  let wantId = '', wantFor = '';
  const expected = () => {   // the fingerprint of the sketch for the current profile (recomputed only when the profile changes)
    const r = getRobot(), key = JSON.stringify(r);
    if (key !== wantFor) { wantFor = key; try { wantId = generateSketch(r, r.hardware).id; } catch (e) { wantId = ''; } }
    return wantId;
  };
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
    stale.hidden = !(getRobot && st === 'ok' && link.sketchId && link.sketchId !== expected());
  };
  tick(); setInterval(tick, 1000);
  if (!getRobot) return pill;
  const both = el('span', { class: 'row', style: 'gap:6px' }, pill, stale);
  return both;
}
