// tracklib.js - the track library: built-in shapes, real tracks from the Simulation-To-Real-Life project
// (web/tracks/*.track), and the student's own saved tracks ("My tracks", kept in this browser; export/import
// as one JSON file to share with the class).
import { parseTrackFile } from './corridor.js';

export const REAL_TRACKS = ['c90_l_20', 'c90_r_20', 'w20_s', 'm_s90_20', 'w20_err_A', 'w20_gaps_A', 'w20_bumps', 'w20_bend_l', 'o_gaps_20', 'smooth_loop', 'tight_broken'];

export async function loadRealTrack(id) {
  const r = await fetch(new URL(`../tracks/${id}.track`, import.meta.url));
  if (!r.ok) throw new Error(`track ${id}: ${r.status}`);
  return parseTrackFile(await r.text());
}

const KEY = 'rt.tracks';
export function myTracks() { try { return JSON.parse(localStorage.getItem(KEY) || '[]'); } catch (e) { return []; } }
function store(list) { try { localStorage.setItem(KEY, JSON.stringify(list)); } catch (e) { /* storage blocked */ } }

/** save the current track settings (env.track) under a name; returns its id */
export function saveMyTrack(name, track) {
  const list = myTracks();
  const id = 'm' + Date.now().toString(36);
  const { preset, points, width_mm, radius_mm, closed, imported } = track;
  list.push({ id, name, track: { preset, points, width_mm, radius_mm, closed, imported } });
  store(list);
  return id;
}
export function deleteMyTrack(id) { store(myTracks().filter((x) => x.id !== id)); }
export function exportMyTracks() { return JSON.stringify({ schema: 'robot-tuner/tracks@1', tracks: myTracks() }, null, 1); }
/** merge tracks from an exported file; returns how many were added */
export function importMyTracks(text) {
  const d = JSON.parse(text);
  if (!d || d.schema !== 'robot-tuner/tracks@1' || !Array.isArray(d.tracks)) throw new Error('not a Robot Tuner track file');
  const list = myTracks(), have = new Set(list.map((x) => x.id));
  let n = 0;
  for (const tr of d.tracks) if (tr && tr.id && tr.track && !have.has(tr.id)) { list.push(tr); n++; }
  store(list);
  return n;
}
