// The replay player: one self-contained HTML page. record.cpp replaces /*__REPLAY_DATA__*/null with the
// run's JSON and __TITLE__ with a title. Pure HTML/CSS/JS, no internet needed to open it.
const char* kPlayerHtml = R"RPLY(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
  :root { --bg:#0f1115; --panel:rgba(22,25,32,.92); --line:#2b303b; --text:#e6e8ee; --muted:#8b93a7; --accent:#7dd3fc;
          --s0:#4cc9f0; --s1:#ffd166; --s2:#f72585; }
  * { box-sizing:border-box; }
  html,body { margin:0; height:100%; background:var(--bg); color:var(--text); font:13px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif; overflow:hidden; }
  canvas { position:fixed; inset:0; width:100%; height:100%; cursor:grab; }
  canvas.drag { cursor:grabbing; }
  .panel { position:fixed; background:var(--panel); border:1px solid var(--line); border-radius:10px; padding:10px 12px; backdrop-filter:blur(4px); }
  #info { top:12px; left:12px; min-width:250px; }
  #info h1 { font-size:14px; margin:0 0 2px; }
  #info .sub { color:var(--muted); margin-bottom:8px; }
  .row { display:flex; justify-content:space-between; gap:12px; }
  .sw { display:inline-block; width:10px; height:10px; border-radius:50%; margin-right:6px; vertical-align:middle; }
  #layers { left:12px; width:250px; }
  #graphs { top:12px; right:12px; bottom:64px; width:392px; overflow-y:auto; padding:8px 10px; }
  #graphs.off { display:none; }
  .g { margin-bottom:8px; }
  .g .hd { display:flex; justify-content:space-between; gap:8px; font-size:12px; margin-bottom:2px; }
  .g .hd b { font-weight:600; }
  .g .hd span { color:var(--muted); font-variant-numeric:tabular-nums; white-space:nowrap; }
  .g canvas { position:static; display:block; width:100%; height:84px; cursor:crosshair; border-radius:6px; background:#0b0d11; }
  .g .lg { font-size:11px; color:var(--muted); display:flex; gap:10px; flex-wrap:wrap; }
  .g .lg i { display:inline-block; width:14px; height:0; border-top:2px solid; vertical-align:middle; margin-right:4px; }
  .g .lg i.d { border-top-style:dashed; }
  #layers h2 { font-size:12px; text-transform:uppercase; letter-spacing:.06em; color:var(--muted); margin:8px 0 4px; }
  #layers h2:first-child { margin-top:0; }
  label { display:flex; align-items:center; gap:7px; padding:2px 0; cursor:pointer; user-select:none; }
  label kbd { margin-left:auto; }
  kbd { font:11px ui-monospace,monospace; color:var(--muted); border:1px solid var(--line); border-radius:4px; padding:0 4px; }
  #bar { left:12px; right:12px; bottom:12px; display:flex; align-items:center; gap:10px; }
  #bar input[type=range] { flex:1; accent-color:var(--accent); }
  button,select { background:#1c2029; color:var(--text); border:1px solid var(--line); border-radius:6px; padding:4px 10px; font:inherit; cursor:pointer; }
  button:hover,select:hover { border-color:var(--accent); }
  #time { font-variant-numeric:tabular-nums; min-width:110px; text-align:right; }
  .outcome-FINISHED { color:#4ade80; } .outcome-CRASHED { color:#f87171; } .outcome-TIMED { color:#fbbf24; }
  #tip { position:fixed; pointer-events:none; display:none; padding:4px 8px; font-size:12px; }
  @media (max-width:900px) { #graphs { width:auto; left:12px; top:auto; height:45%; } }
  @media (max-width:700px) { #layers { display:none; } #info { min-width:0; right:12px; } }
</style>
</head>
<body>
<canvas id="c"></canvas>
<div id="info" class="panel">
  <h1 id="title"></h1>
  <div class="sub" id="sub"></div>
  <div class="row"><span>Time</span><span id="t">0.00 s</span></div>
  <div class="row"><span>State</span><span id="state"></span></div>
  <div class="row"><span>Wheels (m/s)</span><span id="wheels"></span></div>
  <div class="row" id="fitrms_row"><span>Fit rms L / R</span><span id="fitrms"></span></div>
  <div class="row" id="tilt_row" title="fitted line angle minus the angle of the real wall piece under its points"><span>Line tilt L / R</span><span id="tilt"></span></div>
  <div id="readings"></div>
</div>
<div id="layers" class="panel">
  <h2>Show</h2>
  <label><input type="checkbox" id="L_walls" checked> Walls <kbd>W</kbd></label>
  <label><input type="checkbox" id="L_dots" checked> Sensor dots (map) <kbd>D</kbd></label>
  <label><input type="checkbox" id="L_recent"> Only last 3 s of dots <kbd>3</kbd></label>
  <label><input type="checkbox" id="L_labels" checked> Distance labels <kbd>L</kbd></label>
  <label><input type="checkbox" id="L_rays" checked> Sensor rays <kbd>R</kbd></label>
  <label><input type="checkbox" id="L_trail" checked> Trail <kbd>T</kbd></label>
  <label><input type="checkbox" id="L_robot" checked> Robot</label>
  <label id="lines_row"><input type="checkbox" id="L_lines" checked> LineFit's wall lines</label>
  <label id="fitpts_row"><input type="checkbox" id="L_fitpts" checked> LineFit's fit points <kbd>P</kbd></label>
  <label id="fitonly_row"><input type="checkbox" id="L_fitonly"> Fit window only <kbd>O</kbd></label>
  <div id="rs_box" style="display:none">
    <h2>What if faster? (experimental)</h2>
    <select id="rs_sel" style="width:100%"></select>
    <label><input type="radio" name="rs_sync" id="rs_time" checked> sync by time</label>
    <label><input type="radio" name="rs_sync" id="rs_dist"> sync by distance along the track</label>
    <div id="rs_info" style="color:var(--muted);font-size:12px"></div>
  </div>
  <h2>Camera</h2>
  <label><input type="checkbox" id="L_follow"> Follow robot <kbd>F</kbd></label>
  <div style="color:var(--muted);margin-top:6px">Wheel: zoom &middot; drag: pan &middot; double-click: fit</div>
</div>
<div id="graphs" class="panel"></div>
<div id="bar" class="panel">
  <button id="play" title="Space">Play</button>
  <button id="back" title="Left arrow">&#9664;</button>
  <button id="fwd" title="Right arrow">&#9654;</button>
  <input type="range" id="seek" min="0" value="0" step="1">
  <span id="time"></span>
  <select id="speed" title="Playback speed">
    <option value="0.25">0.25&times;</option><option value="0.5">0.5&times;</option>
    <option value="1" selected>1&times;</option><option value="2">2&times;</option><option value="4">4&times;</option>
  </select>
  <button id="gbtn" title="G">Graphs</button>
</div>
<div id="tip" class="panel"></div>
<div id="rs_banner" class="panel" style="display:none;top:12px;left:50%;transform:translateX(-50%);max-width:46%;border-color:#fb923c;color:#fdba74;font-weight:600;text-align:center"></div>
<script>
const R = /*__REPLAY_DATA__*/null;
const T = R.ticks, N = T.length, M = R.meta, RB = R.robot;
const COL = ['--s0','--s1','--s2'].map(v => getComputedStyle(document.documentElement).getPropertyValue(v).trim());
const STATES = ['Navigating','Emergency: stop','Emergency: reverse'];
// The view is drawn y-UP (math frame: +y is the robot's left, angles counter-clockwise), so it is not
// mirrored: a positive mount angle looks left, exactly like on the real robot.
const NAMES = RB.sensor_angles.map(a => Math.abs(a) < 1 ? 'Front' : (a > 0 ? 'Left' : 'Right'));
const FRONT = RB.chassis_front !== undefined ? RB.chassis_front : 0.6 * RB.chassis_length;
const REAR = RB.chassis_rear !== undefined ? RB.chassis_rear : 0.4 * RB.chassis_length;
const SPOS = RB.sensor_pos || [[0,0],[0,0],[0,0]];
const $ = id => document.getElementById(id);
const cv = $('c'), ctx = cv.getContext('2d');

$('title').textContent = `${M.track} · ${M.controller}`;
$('sub').innerHTML = `<span class="outcome-${M.outcome.split(' ')[0]}">${M.outcome}</span> after ${(N*M.dt).toFixed(1)} s · model ${M.model || 'none'} · seed ${M.seed}`;
if (!M.has_lines) $('lines_row').style.display = 'none';
const HAS_PTS = M.has_lines && T.length && T[0].length > 31;  // recordings made before fit points existed lack them
if (!HAS_PTS) { for (const id of ['fitpts_row', 'fitrms_row', 'fitonly_row', 'tilt_row']) $(id).style.display = 'none'; }
$('readings').innerHTML = [0,1,2].map(i => `<div class="row"><span><span class="sw" style="background:${COL[i]}"></span>${NAMES[i]} ToF</span><span id="r${i}"></span></div>`).join('');
$('seek').max = N - 1;

let k = 0, playing = false, speed = 1, acc = 0;
const cam = { x:0, y:0, z:100 };
const on = id => $(id).checked;

// Tick columns: 0 t, 1-3 pose, 4-5 wheel commands, 6 state, 7-9 readings, 10-15 hit points,
// 16-17 encoder speeds, 18-19 ground speeds, 20 heading error (deg), 21 off-centre (m), 22 clearance (m),
// 23-30 LineFit's two wall lines (optional), 31/32 LineFit's own fit points left / right (flat x,y lists, mapped
// onto the true pose like the lines), 33/34 each line's rms to its own points (m), 35/36 window points the outlier
// step dropped (left / right). "Fit window only" (O) hides the sensor map and shows just these, per wall.
const pose = r => [r[1], r[2], r[3]];
const hit = (r, i) => { const b = 10 + 2*i; return r[b] === null ? null : [r[b], r[b+1]]; };

function fit() {
  let x0=1e9, y0=1e9, x1=-1e9, y1=-1e9;
  const add = (x,y) => { x0=Math.min(x0,x); y0=Math.min(y0,y); x1=Math.max(x1,x); y1=Math.max(y1,y); };
  for (const w of R.walls) { add(w[0],w[1]); add(w[2],w[3]); }
  for (const r of T) add(r[1], r[2]);
  const W = viewW(), H = innerHeight;
  cam.x = (x0+x1)/2; cam.y = (y0+y1)/2;
  cam.z = Math.min(W/(x1-x0+0.4), H/(y1-y0+0.4)) * 0.85;
}
// The map is centred in the free space between the left panels and the graphs panel.
const viewL = () => innerWidth > 1100 ? 274 : 0;
const viewR = () => innerWidth - (innerWidth > 900 && !$('graphs').classList.contains('off') ? 416 : 0);
const viewW = () => viewR() - viewL();
const viewC = () => (viewL() + viewR()) / 2;
const sx = x => (x - cam.x) * cam.z + viewC();
const sy = y => innerHeight/2 - (y - cam.y) * cam.z;
const wx = px => (px - viewC()) / cam.z + cam.x;
const wy = py => cam.y - (py - innerHeight/2) / cam.z;

function resize() {
  $('layers').style.top = ($('info').getBoundingClientRect().bottom + 10) + 'px';
  if (typeof gEls !== 'undefined') gEls.forEach(G => G.base = null);
  const d = devicePixelRatio || 1;
  cv.width = innerWidth * d; cv.height = innerHeight * d;
  ctx.setTransform(d,0,0,d,0,0);
  draw();
}

function line(a,b,c,d,col,w) { ctx.strokeStyle=col; ctx.lineWidth=w; ctx.beginPath(); ctx.moveTo(sx(a),sy(b)); ctx.lineTo(sx(c),sy(d)); ctx.stroke(); }

function drawGrid() {
  const step = cam.z > 300 ? 0.1 : cam.z > 60 ? 0.5 : 1;
  ctx.strokeStyle = 'rgba(255,255,255,0.04)'; ctx.lineWidth = 1; ctx.beginPath();
  for (let x = Math.floor(wx(0)/step)*step; x < wx(innerWidth); x += step) { ctx.moveTo(sx(x),0); ctx.lineTo(sx(x),innerHeight); }
  for (let y = Math.floor(wy(innerHeight)/step)*step; y < wy(0); y += step) { ctx.moveTo(0,sy(y)); ctx.lineTo(innerWidth,sy(y)); }
  ctx.stroke();
}

function drawRobot(x, y, th, alpha, body) {
  ctx.save();
  // y-up: rotate by -theta, then flip y (a math-frame robot drawn on a y-down canvas).
  ctx.translate(sx(x), sy(y)); ctx.rotate(-th); ctx.scale(cam.z, -cam.z); ctx.globalAlpha = alpha;
  const L = FRONT + REAR, Wd = RB.chassis_width;
  ctx.fillStyle = body || '#2563eb'; ctx.fillRect(-REAR, -Wd/2, L, Wd);
  ctx.fillStyle = '#9ca3af';
  for (const s of [-1, 1]) ctx.fillRect(-RB.wheel_offset - RB.wheel_length/2, s*RB.track_width/2 - RB.wheel_width/2, RB.wheel_length, RB.wheel_width);
  if (RB.caster_d) { ctx.fillStyle = '#64748b'; ctx.beginPath(); ctx.arc(RB.caster_x, 0, RB.caster_d/2, 0, 7); ctx.fill(); }
  ctx.fillStyle = '#fff'; ctx.beginPath(); ctx.arc(0, 0, Wd*0.05, 0, 7); ctx.fill();  // axle midpoint = pose
  ctx.fillStyle = '#facc15';
  for (let s = 0; s < 3; s++) { ctx.beginPath(); ctx.arc(SPOS[s][0], SPOS[s][1], 0.005, 0, 7); ctx.fill(); }
  ctx.restore();
}

// Fitted line angle minus the angle of the real wall pieces under its points (nearest wall of each point, axial mean).
function lineTilt(r, b, P) {
  if (r[b] === null || r[b] === undefined || !P || P.length < 2) return '-';
  const segd = (x, y, w) => { const vx = w[2]-w[0], vy = w[3]-w[1], L2 = vx*vx+vy*vy;
    const t = L2 > 0 ? Math.max(0, Math.min(1, ((x-w[0])*vx + (y-w[1])*vy) / L2)) : 0;
    return Math.hypot(w[0]+t*vx-x, w[1]+t*vy-y); };
  let c = 0, s = 0;
  for (let i = 0; i + 1 < P.length; i += 2) {
    let best = null, bd = 1e9;
    for (const w of R.walls) { const d = segd(P[i], P[i+1], w); if (d < bd) { bd = d; best = w; } }
    if (best) { const a = Math.atan2(best[3]-best[1], best[2]-best[0]); c += Math.cos(2*a); s += Math.sin(2*a); }
  }
  let d = Math.atan2(r[b+3]-r[b+1], r[b+2]-r[b]) - 0.5 * Math.atan2(s, c);
  while (d > Math.PI/2) d -= Math.PI; while (d < -Math.PI/2) d += Math.PI;
  return (d * 180 / Math.PI).toFixed(2) + '\u00b0';
}

function draw() {
  const r = T[k];
  const [px, py, pth] = pose(r);
  if (on('L_follow')) { cam.x = px; cam.y = py; }
  ctx.fillStyle = getComputedStyle(document.body).backgroundColor; ctx.fillRect(0,0,innerWidth,innerHeight);
  drawGrid();
  const lw = Math.max(1.5, cam.z * 0.006);

  if (on('L_walls')) {
    for (const w of R.walls) line(w[0],w[1],w[2],w[3],'#cfd5e3',lw);
    if (R.finish) { const f = R.finish; line(f[0],f[1],f[2],f[3],'#22c55e',lw*1.6); }
  }
  if (on('L_trail') && !(HAS_PTS && on('L_fitonly'))) {
    ctx.strokeStyle = 'rgba(74,222,128,0.45)'; ctx.lineWidth = 1.5; ctx.beginPath();
    for (let i = 0; i <= k; i++) { const p = pose(T[i]); i ? ctx.lineTo(sx(p[0]),sy(p[1])) : ctx.moveTo(sx(p[0]),sy(p[1])); }
    ctx.stroke();
  }
  if (on('L_dots') && !(HAS_PTS && on('L_fitonly'))) {
    const from = on('L_recent') ? Math.max(0, k - Math.round(3 / M.dt)) : 0;
    const rad = Math.max(1.6, Math.min(4, cam.z * 0.008));
    for (let s = 0; s < 3; s++) {
      ctx.fillStyle = COL[s]; ctx.globalAlpha = 0.75;
      for (let i = from; i <= k; i++) { const h = hit(T[i], s); if (h) { ctx.beginPath(); ctx.arc(sx(h[0]), sy(h[1]), rad, 0, 7); ctx.fill(); } }
    }
    ctx.globalAlpha = 1;
  }
  const FONLY = HAS_PTS && on('L_fitonly');
  if (M.has_lines && (on('L_lines') || FONLY)) {
    // Fit-window mode: each wall's line in the colour of its own points (left yellow, right magenta).
    [[23, '#facc15'], [27, '#e879f9']].forEach(([b, c]) => {
      if (r[b] !== null && r[b] !== undefined) line(r[b], r[b+1], r[b+2], r[b+3], FONLY ? c : '#38bdf8', lw*1.3);
    });
  }
  if (FONLY) {  // dropped window points: grey crosses
    ctx.strokeStyle = '#94a3b8'; ctx.lineWidth = 1.5;
    for (const b of [35, 36]) { const P = r[b] || [];
      for (let i = 0; i + 1 < P.length; i += 2) { const X = sx(P[i]), Y = sy(P[i+1]);
        ctx.beginPath(); ctx.moveTo(X-3,Y-3); ctx.lineTo(X+3,Y+3); ctx.moveTo(X-3,Y+3); ctx.lineTo(X+3,Y-3); ctx.stroke(); } }
  }
  // LineFit's own window points: hollow squares (left yellow, right magenta), unlike the filled sensor dots.
  if (HAS_PTS && (on('L_fitpts') || on('L_fitonly'))) {
    [[31, '#facc15'], [32, '#e879f9']].forEach(([b, c]) => {
      const P = r[b] || [];
      ctx.strokeStyle = c; ctx.lineWidth = 1.5;
      for (let i = 0; i + 1 < P.length; i += 2) ctx.strokeRect(sx(P[i]) - 3, sy(P[i+1]) - 3, 6, 6);
    });
  }
  for (let s = 0; s < 3; s++) {
    const h = hit(r, s);
    const a = pth + RB.sensor_angles[s] * Math.PI / 180;
    // Each ray starts at its own sensor, not at the robot origin.
    const ox = px + Math.cos(pth) * SPOS[s][0] - Math.sin(pth) * SPOS[s][1];
    const oy = py + Math.sin(pth) * SPOS[s][0] + Math.cos(pth) * SPOS[s][1];
    const end = h || [ox + Math.cos(a) * 0.5, oy + Math.sin(a) * 0.5];
    if (on('L_rays')) {
      ctx.setLineDash(h ? [] : [4,4]);
      line(ox, oy, end[0], end[1], COL[s] + (h ? 'cc' : '55'), 1.5);
      ctx.setLineDash([]);
    }
    if (h) {
      ctx.fillStyle = COL[s]; ctx.beginPath(); ctx.arc(sx(h[0]), sy(h[1]), 5, 0, 7); ctx.fill();
      ctx.strokeStyle = '#000'; ctx.lineWidth = 1; ctx.stroke();
      if (on('L_labels') && !(HAS_PTS && on('L_fitonly'))) {
        const txt = `${(r[7+s]*100).toFixed(1)} cm`;
        ctx.font = '600 12px system-ui,sans-serif';
        const tw = ctx.measureText(txt).width, X = sx(h[0]) + 8, Y = sy(h[1]) - 8;
        ctx.fillStyle = 'rgba(0,0,0,0.7)'; ctx.fillRect(X-3, Y-12, tw+6, 16);
        ctx.fillStyle = COL[s]; ctx.fillText(txt, X, Y);
      }
    }
  }
  if (RSEL >= 0) drawResim(k);
  if (on('L_robot')) {
    drawRobot(px, py, pth, 1);
  }

  rsBanner();
  $('t').textContent = `${r[0].toFixed(2)} s`;
  $('time').textContent = `${r[0].toFixed(2)} / ${T[N-1][0].toFixed(2)} s`;
  $('state').textContent = STATES[r[6]] || r[6];
  if (HAS_PTS) $('tilt').textContent = [[23, 31], [27, 32]].map(([b, pb]) => lineTilt(r, b, r[pb])).join(' / ');
  if (HAS_PTS) $('fitrms').textContent = [33, 34].map(b => r[b] === null ? '-' : (r[b]*1000).toFixed(1) + ' mm').join(' / ');
  $('wheels').textContent = `L ${r[4].toFixed(2)}  R ${r[5].toFixed(2)}`;
  for (let s = 0; s < 3; s++) $('r'+s).textContent = r[7+s] >= RB.max_range - 0.01 ? 'no return' : `${(r[7+s]*100).toFixed(1)} cm`;
  $('seek').value = k;
  drawGraphs();
  $('play').textContent = playing ? 'Pause' : (k >= N-1 ? 'Replay' : 'Play');
}

// ---------------------------------------------------------------- "what if faster" re-simulation (experimental)
// record --resim / --ramp re-ran this episode (same track, start pose, noise seed) in the SIMULATOR at another top
// speed. Shown as an orange ghost robot + path, synced to the recording by time or by distance along the track.
const RS = R.resims || [];
let RSEL = -1;
const RS_SMAX = RS.map(S => { let m = -1e9; return S.ticks.map(q => (m = Math.max(m, q[4]))); });
if (RS.length) {
  $('rs_box').style.display = '';
  $('rs_sel').innerHTML = '<option value="-1">off</option>' + RS.map((S, i) => `<option value="${i}">${S.label} - ${S.outcome}</option>`).join('');
  $('rs_sel').addEventListener('change', e => { RSEL = +e.target.value; draw(); });
  for (const id of ['rs_time', 'rs_dist']) $(id).addEventListener('change', draw);
}
function rsIndex(k) {
  const S = RS[RSEL], n = S.ticks.length;
  if (!$('rs_dist').checked || !R.orig_s) return Math.min(n - 1, Math.max(0, Math.round(T[k][0] / M.dt)));
  const s = R.orig_s[Math.min(k, R.orig_s.length - 1)], A = RS_SMAX[RSEL];
  let lo = 0, hi = n - 1;
  while (lo < hi) { const m = (lo + hi) >> 1; if (A[m] >= s) hi = m; else lo = m + 1; }
  return lo;
}
function drawResim(k) {
  const S = RS[RSEL], TT = S.ticks, j = rsIndex(k), q = TT[j];
  const path = (a, b, col, w) => { ctx.strokeStyle = col; ctx.lineWidth = w; ctx.beginPath();
    for (let i = a; i <= b; i++) i === a ? ctx.moveTo(sx(TT[i][1]), sy(TT[i][2])) : ctx.lineTo(sx(TT[i][1]), sy(TT[i][2])); ctx.stroke(); };
  path(0, TT.length - 1, 'rgba(251,146,60,0.30)', 1.5);
  path(0, j, 'rgba(251,146,60,0.95)', 2);
  ctx.font = '600 12px system-ui,sans-serif';
  if (S.split) {
    const X = sx(S.split.x), Y = sy(S.split.y);
    ctx.strokeStyle = '#fde047'; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(X, Y, 8, 0, 7); ctx.stroke();
    ctx.fillStyle = '#fde047'; ctx.fillText(`paths split > 2 cm (re-sim ${S.split.t.toFixed(1)} s)`, X + 11, Y - 10);
  }
  if (S.outcome === 'CRASHED') {
    const e = TT[TT.length - 1], X = sx(e[1]), Y = sy(e[2]);
    ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(X-8,Y-8); ctx.lineTo(X+8,Y+8); ctx.moveTo(X-8,Y+8); ctx.lineTo(X+8,Y-8); ctx.stroke();
    ctx.fillStyle = '#ef4444'; ctx.fillText(`re-sim crash at ${e[0].toFixed(1)} s`, X + 11, Y + 16);
  }
  drawRobot(q[1], q[2], q[3], 0.55, '#f97316');
  const r = T[k], gap = Math.hypot(q[1] - r[1], q[2] - r[2]);
  const ended = (j === TT.length - 1 && (T[k][0] > q[0] + 1e-6 || (R.orig_s && $('rs_dist').checked))) ? ` (re-sim ${S.outcome.toLowerCase()} at ${q[0].toFixed(1)} s)` : '';
  $('rs_info').textContent = `re-sim t ${q[0].toFixed(2)} s · ${q[5].toFixed(3)} m/s · ${(gap * 100).toFixed(1)} cm from the recording${ended}`;
}
function rsBanner() {
  const b = $('rs_banner');
  if (RSEL < 0) { b.style.display = 'none'; return; }
  b.style.display = '';
  b.textContent = `RE-SIMULATION, not a recording: ${RS[RSEL].label} (orange). ` + (R.resim_note || 'Same track, start pose and noise seed, simulator.');
}

let last = null;
function loop(ts) {
  if (playing) {
    if (last !== null) acc += (ts - last) / 1000 * speed;
    while (acc >= M.dt && k < N-1) { k++; acc -= M.dt; }
    if (k >= N-1) playing = false;
    draw();
  }
  last = ts;
  requestAnimationFrame(loop);
}

function setK(v) { k = Math.max(0, Math.min(N-1, v)); acc = 0; draw(); }
$('play').onclick = () => { if (k >= N-1) k = 0; playing = !playing; acc = 0; draw(); };
$('back').onclick = () => { playing = false; setK(k-1); };
$('fwd').onclick = () => { playing = false; setK(k+1); };
$('seek').oninput = e => setK(+e.target.value);
$('speed').onchange = e => { speed = +e.target.value; };
document.querySelectorAll('#layers input').forEach(el => el.addEventListener('change', draw));
const toggle = id => { $(id).checked = !$(id).checked; draw(); };
addEventListener('keydown', e => {
  if (e.target.tagName === 'SELECT') return;
  const key = e.key.toLowerCase();
  if (key === ' ') { e.preventDefault(); $('play').click(); }
  else if (e.key === 'ArrowLeft') { playing = false; setK(k - (e.shiftKey ? 20 : 1)); }
  else if (e.key === 'ArrowRight') { playing = false; setK(k + (e.shiftKey ? 20 : 1)); }
  else if (key === 'w') toggle('L_walls');
  else if (key === 'p' && HAS_PTS) toggle('L_fitpts');
  else if (key === 'o' && HAS_PTS) toggle('L_fitonly');
  else if (key === 'd') toggle('L_dots');
  else if (key === '3') toggle('L_recent');
  else if (key === 'l') toggle('L_labels');
  else if (key === 'r') toggle('L_rays');
  else if (key === 't') toggle('L_trail');
  else if (key === 'f') toggle('L_follow');
  else if (key === 'g') toggleGraphs();
  else if (key === 'x' && RS.length) { const o = $('rs_sel'); o.selectedIndex = (o.selectedIndex + 1) % o.options.length; o.dispatchEvent(new Event('change')); }
});

// Zoom around the cursor, drag to pan, double-click to fit.
cv.addEventListener('wheel', e => {
  e.preventDefault();
  const x = wx(e.clientX), y = wy(e.clientY);
  cam.z = Math.max(5, Math.min(5000, cam.z * Math.pow(1.0015, -e.deltaY)));
  if (!on('L_follow')) { cam.x = x - (e.clientX - viewC()) / cam.z; cam.y = y + (e.clientY - innerHeight/2) / cam.z; }
  draw();
}, { passive:false });
let drag = null;
cv.addEventListener('pointerdown', e => { drag = { x:e.clientX, y:e.clientY, cx:cam.x, cy:cam.y }; cv.classList.add('drag'); cv.setPointerCapture(e.pointerId); });
cv.addEventListener('pointermove', e => {
  if (drag) {
    $('L_follow').checked = false;
    cam.x = drag.cx - (e.clientX - drag.x) / cam.z; cam.y = drag.cy + (e.clientY - drag.y) / cam.z; draw();
    return;
  }
  // Hover: the nearest sensor dot shows when and what it read.
  if (!on('L_dots')) { $('tip').style.display = 'none'; return; }
  let best = null, bd = 64;
  const from = on('L_recent') ? Math.max(0, k - Math.round(3 / M.dt)) : 0;
  for (let i = from; i <= k; i++) for (let s = 0; s < 3; s++) {
    const h = hit(T[i], s); if (!h) continue;
    const d = (sx(h[0]) - e.clientX) ** 2 + (sy(h[1]) - e.clientY) ** 2;
    if (d < bd) { bd = d; best = [i, s]; }
  }
  const tip = $('tip');
  if (best) {
    const [i, s] = best;
    tip.innerHTML = `<span class="sw" style="background:${COL[s]}"></span>${NAMES[s]}: ${(T[i][7+s]*100).toFixed(1)} cm at ${T[i][0].toFixed(2)} s`;
    tip.style.left = (e.clientX + 14) + 'px'; tip.style.top = (e.clientY + 14) + 'px'; tip.style.display = 'block';
  } else tip.style.display = 'none';
});
cv.addEventListener('pointerup', () => { drag = null; cv.classList.remove('drag'); });
cv.addEventListener('dblclick', () => { $('L_follow').checked = false; fit(); draw(); });
addEventListener('resize', resize);


// ------------------------------------------------------------------------------------------- graphs
// Each graph shows the whole run; the white line is "now". Click or drag on a graph to jump there.
const CL = '#4cc9f0', CR = '#f72585';
const pct = v => v / RB.max_speed * 100, cm = v => v * 100;
const GRAPHS = [
  { title:'Motor command (PWM, % of max speed)', unit:'%', zero:true, fmt:v => v.toFixed(0) + '%',
    series:[{ c:4, f:pct, col:CL, name:'Left' }, { c:5, f:pct, col:CR, name:'Right' }] },
  { title:'Wheel speed (m/s): encoders vs ground', zero:true, fmt:v => v.toFixed(2),
    series:[{ c:16, col:CL, name:'Left encoder' }, { c:17, col:CR, name:'Right encoder' },
            { c:18, col:CL, name:'Left ground', dash:true }, { c:19, col:CR, name:'Right ground', dash:true }] },
  { title:'ToF readings (cm)', fmt:v => v >= RB.max_range*100 - 1 ? 'none' : v.toFixed(1), cap:true,
    series:[0,1,2].map(i => ({ c:7+i, f:cm, col:COL[i], name:NAMES[i] })) },
  { title:'Heading error vs walls (°)', zero:true, fmt:v => v.toFixed(1) + '°',
    series:[{ c:20, col:'#a3e635', name:'+ = turned right' }] },
  { title:'Off-centre (cm)', zero:true, fmt:v => v.toFixed(1),
    series:[{ c:21, f:cm, col:'#fbbf24', name:'+ = closer to the left wall' }] },
  { title:'Clearance: footprint to nearest wall (cm)', zero:true, fmt:v => v.toFixed(1),
    series:[{ c:22, f:cm, col:'#f87171', name:'0 = touching' }] },
];
// A real-robot replay (tools/log_to_replay.py) has no ground truth: hide those series/graphs.
if (M.real) {
  for (const g of GRAPHS) g.series = g.series.filter(s => s.c < 18 || s.c > 22);
  for (let i = GRAPHS.length - 1; i >= 0; i--) if (!GRAPHS[i].series.length) GRAPHS.splice(i, 1);
}
const gEls = [];
function buildGraphs() {
  const host = $('graphs');
  host.innerHTML = '';
  GRAPHS.forEach((g, gi) => {
    const vals = g.series.map(sr => T.map(r => r[sr.c] === null || r[sr.c] === undefined ? NaN : (sr.f ? sr.f(r[sr.c]) : r[sr.c])));
    let lo = Infinity, hi = -Infinity;
    const all = vals.flat().filter(Number.isFinite).sort((a, b) => a - b);
    if (all.length) { lo = all[0]; hi = all[all.length - 1]; }
    if (g.cap && all.length) hi = Math.max(30, all[Math.floor(all.length * 0.95)] * 1.15);  // 4 m "no return" would flatten the rest
    if (g.zero) { lo = Math.min(lo, 0); hi = Math.max(hi, 0); }
    if (!(hi > lo)) { lo -= 1; hi += 1; }
    const pad = (hi - lo) * 0.08; lo -= g.cap ? 0 : pad; hi += pad;
    const d = document.createElement('div'); d.className = 'g';
    d.innerHTML = `<div class="hd"><b>${g.title}</b></div><canvas></canvas><div class="lg">${g.series.map(sr =>
      `<span><i class="${sr.dash ? 'd' : ''}" style="border-color:${sr.col}"></i>${sr.name} <b data-v></b></span>`).join('')}</div>`;
    host.appendChild(d);
    const c = d.querySelector('canvas');
    const seekTo = e => { const b = c.getBoundingClientRect(); playing = false; setK(Math.round((e.clientX - b.left) / b.width * (N - 1))); };
    c.addEventListener('pointerdown', e => { c.setPointerCapture(e.pointerId); seekTo(e); c.onpointermove = seekTo; });
    c.addEventListener('pointerup', () => { c.onpointermove = null; });
    gEls.push({ g, c, vals, lo, hi, base:null, vEls:[...d.querySelectorAll('[data-v]')] });
  });
}
function renderGraphBase(G) {
  const d = devicePixelRatio || 1, w = G.c.clientWidth, h = G.c.clientHeight;
  if (!w) return;
  G.c.width = w * d; G.c.height = h * d;
  const off = document.createElement('canvas'); off.width = w * d; off.height = h * d;
  const x = off.getContext('2d'); x.scale(d, d);
  const X = i => i / Math.max(1, N - 1) * w, Y = v => h - 4 - (Math.min(v, G.hi) - G.lo) / (G.hi - G.lo) * (h - 8);
  x.fillStyle = 'rgba(255,255,255,0.35)'; x.font = '10px system-ui,sans-serif';
  x.fillText(G.hi.toFixed(G.hi - G.lo < 5 ? 2 : 0), 3, 11); x.fillText(G.lo.toFixed(G.hi - G.lo < 5 ? 2 : 0), 3, h - 3);
  if (G.lo < 0 && G.hi > 0) { x.strokeStyle = 'rgba(255,255,255,0.18)'; x.setLineDash([3,3]); x.beginPath(); x.moveTo(0, Y(0)); x.lineTo(w, Y(0)); x.stroke(); x.setLineDash([]); }
  G.vals.forEach((v, si) => {
    const sr = G.g.series[si];
    x.strokeStyle = sr.col; x.lineWidth = 1.3; x.setLineDash(sr.dash ? [4,3] : []); x.globalAlpha = sr.dash ? 0.75 : 0.95;
    x.beginPath(); let pen = false;
    for (let i = 0; i < N; i++) { if (!Number.isFinite(v[i])) { pen = false; continue; } pen ? x.lineTo(X(i), Y(v[i])) : x.moveTo(X(i), Y(v[i])); pen = true; }
    x.stroke();
  });
  x.globalAlpha = 1; x.setLineDash([]);
  if (M.outcome === 'CRASHED') { x.fillStyle = 'rgba(248,113,113,0.8)'; x.fillRect(w - 2, 0, 2, h); }
  G.base = off;
}
function drawGraphs() {
  if ($('graphs').classList.contains('off')) return;
  for (const G of gEls) {
    if (!G.base) renderGraphBase(G);
    if (!G.base) continue;
    const ctx2 = G.c.getContext('2d'), d = devicePixelRatio || 1, w = G.c.width / d, h = G.c.height / d;
    ctx2.setTransform(1,0,0,1,0,0); ctx2.clearRect(0, 0, G.c.width, G.c.height); ctx2.drawImage(G.base, 0, 0);
    ctx2.setTransform(d,0,0,d,0,0);
    const cx = k / Math.max(1, N - 1) * w;
    ctx2.strokeStyle = '#fff'; ctx2.lineWidth = 1; ctx2.beginPath(); ctx2.moveTo(cx, 0); ctx2.lineTo(cx, h); ctx2.stroke();
    G.vals.forEach((v, si) => { G.vEls[si].textContent = Number.isFinite(v[k]) ? G.g.fmt(v[k]) : '–'; });
  }
}
function toggleGraphs() {
  $('graphs').classList.toggle('off');
  gEls.forEach(G => G.base = null);
  draw();
}
$('gbtn').onclick = toggleGraphs;

if (innerWidth < 700) $('graphs').classList.add('off');
buildGraphs(); fit(); resize(); requestAnimationFrame(loop);
</script>
</body>
</html>
)RPLY";
