// save-reminder.js - keep students from losing their robot: changes since the last saved robot file make the browser
// ask "Leave site?" on refresh / close (browsers only allow their own box there: no custom text, no download), and the
// Measure page offers "Save your work" after every 3 finished measurements.
import { el, t, download, toast } from './common.js';
import { robotToFile } from './robotfile.js';
import { downloadName, isDirty, markSaved } from './store.js';

/** download the robot file now and count it as saved */
export function saveRobotFile(robot) {
  download(downloadName(robot, '.robot.txt'), robotToFile(robot));
  markSaved();
  toast(t('save.done'), 'ok');
}

/** refresh / close / leave with unsaved changes: the browser's own "Leave site?" box ("Leave" = go on without saving) */
export function guardLeave() {
  let inside = false;   // a click on a link to another Robot Tuner page: the robot is already saved in this browser
  document.addEventListener('click', (e) => {
    const a = e.target.closest && e.target.closest('a[href]');
    if (a && !a.hasAttribute('download') && new URL(a.href, location.href).origin === location.origin) { inside = true; setTimeout(() => { inside = false; }, 2000); }
  }, true);
  window.addEventListener('beforeunload', (e) => { if (!inside && isDirty()) { e.preventDefault(); e.returnValue = ''; } });
}

const finished = new Set();
/** call when a measurement step got its value (measured or typed; each step counts once); every 3rd step a small
 *  "Save your work" window */
export function measurementDone(getRobot, step) {
  if (finished.has(step)) return;
  finished.add(step);
  if (finished.size % 3 !== 0 || !isDirty()) return;
  const dlg = el('dialog', { class: 'cmd-dialog save-dialog' });
  const close = () => { dlg.close(); dlg.remove(); };
  dlg.append(
    el('h3', { text: t('save.h') }),
    el('p', { text: t('save.p', { n: finished.size }) }),
    el('div', { class: 'row' },
      el('button', { class: 'primary', text: t('save.btn'), onclick: () => { saveRobotFile(getRobot()); close(); } }),
      el('button', { text: t('save.later'), onclick: close })));
  dlg.addEventListener('cancel', (e) => { e.preventDefault(); close(); });
  document.body.append(dlg); dlg.showModal();
}
