// install.js - ESP32 without the Arduino IDE: flash the Robot Tuner install firmware from the browser (esptool-js over
// Web Serial), then send the robot's settings (the robot file's keys) with SET ... and SAVE.
import { el, t, toast } from './common.js';
import { serialSupported, RobotLink } from './serial.js';
import { robotToFile } from './robotfile.js';
import { FW_KEYS } from './firmware-esp32.js';
import { btName, normalizeHardware } from './firmware-gen.js';

export const IMAGE_URL = 'firmware/esp32/robot_tuner_esp32.bin';

/** the SET lines for this robot: every firmware key that is set in the robot file (N/A lines are skipped) */
export function settingsLines(robot) {
  const keys = new Set(FW_KEYS), out = [];
  for (const raw of robotToFile(robot).split('\n')) {
    const line = raw.replace(/#.*/, '').trim();
    if (!line) continue;
    const [k, v] = line.split(/\s+/);
    if (keys.has(k) && v !== undefined && v !== 'N/A') out.push(`SET ${k} ${v}`);
  }
  const hw = normalizeHardware(robot.hardware, robot.sensors.length);
  if (hw.bt !== false) out.push(`SET bluetooth_name ${btName(robot, hw)}`);   // the name the student pairs with
  return out;
}

async function flash(log, progress) {
  const { ESPLoader, Transport } = await import('../vendor/esptool-js/esptool.bundle.js');
  const res = await fetch(IMAGE_URL);
  if (!res.ok) throw new Error(t('inst.no_image'));
  const data = new Uint8Array(await res.arrayBuffer());
  const port = await navigator.serial.requestPort();
  const transport = new Transport(port, false);
  try {
    const loader = new ESPLoader({ transport, baudrate: 460800, terminal: { clean() {}, writeLine: (s) => log(s), write: (s) => log(s) } });
    const chip = await loader.main();
    log(t('inst.chip', { chip }));
    if (/S2|S3|C2|C3|C5|C6|H2|P4/i.test(chip)) throw new Error(t('inst.wrong_chip', { chip }));
    await loader.writeFlash({ fileArray: [{ data, address: 0 }], flashMode: 'keep', flashFreq: 'keep', flashSize: 'keep', eraseAll: false, compress: true,
      reportProgress: (i, written, total) => progress(written / total) });
    await loader.after('hard_reset');
  } finally {
    try { await transport.disconnect(); } catch (e) { /* already closed */ }
  }
}

async function sendSettings(robot, log) {
  const link = new RobotLink();
  await link.connect();
  try {
    await new Promise((r) => setTimeout(r, 1500));   // the ESP32 restarts when the port opens
    for (const line of settingsLines(robot)) {
      const k = line.split(' ')[1];
      const ok = link.waitFor((l) => (l.startsWith('OK SET ' + k) || l.startsWith('ERR')) && l, 3000);   // listen first: the answer can be immediate
      await link.send(line);
      const ans = await ok;
      if (ans.startsWith('ERR')) throw new Error(ans);
    }
    const saved = link.waitFor((l) => l.startsWith('OK saved'), 5000);
    await link.send('SAVE');
    await saved;
    log(t('inst.saved'));
  } finally { await link.disconnect(); }
}

/** the card on the Firmware tab (ESP32 only) */
export function installCard(getRobot) {
  const logBox = el('pre', { class: 'code own-log', hidden: true });
  const log = (s) => { logBox.hidden = false; logBox.textContent += String(s).replace(/\r/g, '') + (String(s).endsWith('\n') ? '' : '\n'); logBox.scrollTop = logBox.scrollHeight; };
  const bar = el('progress', { max: 1, value: 0, hidden: true, style: 'width:100%' });
  const run = (fn) => async () => {
    btns.forEach((b) => { b.disabled = true; });
    try { await fn(); } catch (e) { if (e && e.name !== 'NotFoundError') { log('ERROR: ' + (e.message || e)); toast(e.message || String(e), 'bad'); } }
    finally { btns.forEach((b) => { b.disabled = !serialSupported(); }); }
  };
  const b1 = el('button', { class: 'primary', text: t('inst.flash'), onclick: run(async () => {
    bar.hidden = false; bar.value = 0; log(t('inst.flashing'));
    await flash(log, (p) => { bar.value = p; });
    log(t('inst.flashed')); toast(t('inst.flashed'), 'ok');
  }) });
  const b2 = el('button', { text: t('inst.send'), onclick: run(async () => { log(t('inst.sending')); await sendSettings(getRobot(), log); toast(t('inst.saved'), 'ok'); }) });
  const btns = [b1, b2];
  if (!serialSupported()) btns.forEach((b) => { b.disabled = true; });
  return el('div', { class: 'card stack install-card' },
    el('h3', { text: t('inst.h') }),
    el('p', { text: t('inst.p') }),
    el('ol', {}, el('li', {}, el('span', { text: t('inst.s1') + ' ' }), b1), el('li', {}, el('span', { text: t('inst.s2') + ' ' }), b2), el('li', { text: t('inst.s3') })),
    bar, logBox,
    el('small', { class: 'muted', text: t(serialSupported() ? 'inst.note' : 'own.nosupport') }));
}
