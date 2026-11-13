import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { FlightSystem, DT, ARENA, U_MAX } from './core.js';

const $ = (id) => document.getElementById(id);
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

// world (x east, y north, z up)  →  three.js (x, y up, -z)
const toThree = (p, out = new THREE.Vector3()) => out.set(p[0], p[2], -p[1]);

// ======================================================================== scene
const canvas = $('scene');
const viewport = canvas.parentElement;
// Without WebGL (old phones, locked-down browsers) the simulation, panels and charts still run; only the 3D view is replaced.
let renderer;
try { renderer = new THREE.WebGLRenderer({ canvas, antialias: true }); }
catch (e) {
  renderer = { setPixelRatio() {}, setClearColor() {}, setSize() {}, render() {} };
  const msg = document.createElement('div'); msg.className = 'nogl';
  msg.textContent = 'This browser could not start WebGL, so the 3D view is off. The controller, wind and learning loop are still running: watch the panels below.';
  viewport.appendChild(msg);
}
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setClearColor(0x0b0f14);

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x0b0f14, 70, 140);
const camera = new THREE.PerspectiveCamera(48, 1, 0.1, 400);
camera.position.set(30, 24, 36);
const controls = new OrbitControls(camera, canvas);
controls.target.set(0, 5, 0);
controls.enableDamping = true;
controls.maxPolarAngle = Math.PI * 0.49;
controls.minDistance = 8; controls.maxDistance = 110;

scene.add(new THREE.HemisphereLight(0xcfe3ff, 0x1a1f26, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 1.4); sun.position.set(20, 40, 10); scene.add(sun);

// ground + arena
const half = ARENA.half;
const ground = new THREE.Mesh(new THREE.PlaneGeometry(half * 2, half * 2), new THREE.MeshStandardMaterial({ color: 0x111821, roughness: 1 }));
ground.rotation.x = -Math.PI / 2; scene.add(ground);
const grid = new THREE.GridHelper(half * 2, 20, 0x2f3b4a, 0x1d2631); grid.position.y = 0.01; scene.add(grid);
const arena = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(half * 2, ARENA.zMax, half * 2)),
  new THREE.LineBasicMaterial({ color: 0x3a4757, transparent: true, opacity: 0.55 }));
arena.position.y = ARENA.zMax / 2; scene.add(arena);

function labelSprite(text, color = '#93a0b0', size = 64) {
  const c = document.createElement('canvas'); c.width = c.height = 128;
  const g = c.getContext('2d'); g.font = `600 ${size}px IBM Plex Sans, sans-serif`;
  g.fillStyle = color; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(text, 64, 68);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthWrite: false, transparent: true }));
  s.scale.set(2.6, 2.6, 1); return s;
}
for (const [t, p] of [['N', [0, half + 2.5, 0.3]], ['E', [half + 2.5, 0, 0.3]], ['S', [0, -half - 2.5, 0.3]], ['W', [-half - 2.5, 0, 0.3]]]) {
  const s = labelSprite(t); toThree(p, s.position); scene.add(s);
}

// drone
const drone = new THREE.Group();
const tilt = new THREE.Group(); drone.add(tilt);
const bodyMat = new THREE.MeshStandardMaterial({ color: 0xdfe6ee, roughness: 0.45, metalness: 0.2 });
const darkMat = new THREE.MeshStandardMaterial({ color: 0x2a323d, roughness: 0.6 });
tilt.add(new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.28, 0.6), bodyMat));
const rotors = [];
for (const [sx, sz] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
  const arm = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 1.5), darkMat);
  arm.rotation.z = Math.PI / 2; arm.rotation.y = Math.atan2(sz, sx) * -1;
  arm.position.set(sx * 0.5, 0.02, sz * 0.5); tilt.add(arm);
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.18), darkMat);
  hub.position.set(sx * 0.95, 0.1, sz * 0.95); tilt.add(hub);
  const disc = new THREE.Mesh(new THREE.CylinderGeometry(0.55, 0.55, 0.02, 28),
    new THREE.MeshBasicMaterial({ color: 0x9fb4c8, transparent: true, opacity: 0.22, depthWrite: false }));
  disc.position.set(sx * 0.95, 0.2, sz * 0.95); tilt.add(disc);
  const blade = new THREE.Mesh(new THREE.BoxGeometry(1.0, 0.02, 0.07), darkMat);
  blade.position.copy(disc.position); tilt.add(blade); rotors.push(blade);
}
const led = new THREE.Mesh(new THREE.SphereGeometry(0.09, 12, 8), new THREE.MeshBasicMaterial({ color: 0x38c6f0 }));
led.position.set(0.47, 0.05, 0); tilt.add(led);
drone.scale.setScalar(1.7);
scene.add(drone);

// home pad at the origin: where the drone returns to swap its battery
const padTex = (() => {
  const c = document.createElement('canvas'); c.width = c.height = 256; const g = c.getContext('2d');
  g.strokeStyle = '#38c6f0'; g.lineWidth = 10; g.beginPath(); g.arc(128, 128, 112, 0, Math.PI * 2); g.stroke();
  g.fillStyle = '#38c6f0'; g.font = '700 150px IBM Plex Sans, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText('H', 128, 138);
  return new THREE.CanvasTexture(c);
})();
const pad = new THREE.Mesh(new THREE.PlaneGeometry(3.2, 3.2), new THREE.MeshBasicMaterial({ map: padTex, transparent: true, opacity: 0.75, depthWrite: false }));
pad.rotation.x = -Math.PI / 2; pad.position.y = 0.025; scene.add(pad);

const shadow = new THREE.Mesh(new THREE.CircleGeometry(1, 32), new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.45, depthWrite: false }));
shadow.rotation.x = -Math.PI / 2; shadow.position.y = 0.02; scene.add(shadow);
const plumbGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
const plumb = new THREE.Line(plumbGeo, new THREE.LineDashedMaterial({ color: 0x5d6b7c, dashSize: 0.4, gapSize: 0.3 }));
scene.add(plumb);

// target
const target = new THREE.Group();
const targetBall = new THREE.Mesh(new THREE.SphereGeometry(0.45, 24, 16), new THREE.MeshBasicMaterial({ color: 0xf5a524 }));
const targetHalo = new THREE.Mesh(new THREE.SphereGeometry(0.9, 24, 16), new THREE.MeshBasicMaterial({ color: 0xf5a524, transparent: true, opacity: 0.18, depthWrite: false }));
target.add(targetBall, targetHalo); scene.add(target);
const targetRing = new THREE.Mesh(new THREE.RingGeometry(0.8, 1.05, 40), new THREE.MeshBasicMaterial({ color: 0xf5a524, transparent: true, opacity: 0.8, side: THREE.DoubleSide }));
targetRing.rotation.x = -Math.PI / 2; targetRing.position.y = 0.03; scene.add(targetRing);
const stemGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
const stem = new THREE.Line(stemGeo, new THREE.LineDashedMaterial({ color: 0xf5a524, dashSize: 0.35, gapSize: 0.35, transparent: true, opacity: 0.7 }));
scene.add(stem);

// flown path + MPC plan
const TRAIL_MAX = 900;
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(TRAIL_MAX * 3), 3));
const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0xb8c2cf, transparent: true, opacity: 0.45 }));
trail.frustumCulled = false; scene.add(trail);
let trailN = 0;
const predGeo = new THREE.BufferGeometry();
predGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(32 * 3), 3));
const pred = new THREE.Line(predGeo, new THREE.LineBasicMaterial({ color: 0x38c6f0 }));
pred.frustumCulled = false; scene.add(pred);
const predDots = new THREE.Points(predGeo, new THREE.PointsMaterial({ color: 0x38c6f0, size: 0.28 }));
predDots.frustumCulled = false; scene.add(predDots);

// wind streaks
const NP = window.matchMedia('(max-width: 640px)').matches ? 380 : 700;
const windPts = Array.from({ length: NP }, () => [rand(-half - 4, half + 4), rand(-half - 4, half + 4), rand(0.3, ARENA.zMax + 2), Math.random()]);
const windGeo = new THREE.BufferGeometry();
windGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(NP * 6), 3));
const windCol = new Float32Array(NP * 6);
for (let i = 0; i < NP; i++) { const b = 0.02 + 0.05 * windPts[i][3]; windCol.set([0.42 * b, 0.62 * b, 0.78 * b, 0, 0, 0], i * 6); }
windGeo.setAttribute('color', new THREE.BufferAttribute(windCol, 3));
const windLines = new THREE.LineSegments(windGeo, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, blending: THREE.AdditiveBlending, depthWrite: false }));
windLines.frustumCulled = false; scene.add(windLines);
const windArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1, 0x7fb6dd, 0.6, 0.4);
scene.add(windArrow);
function rand(a, b) { return a + Math.random() * (b - a); }

function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h, false); camera.aspect = w / h;
  camera.fov = w < 520 ? 60 : 48; camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport); resize();

// ======================================================================== simulation + UI state
let sys, prevP = [0, 0, 0], prevAtt = [0, 0], last = null, speed = 1, acc = 0, tourTimer = null, visible = true;
const toastEl = $('toast'); let toastTimer;
function toast(html, ms = 3800) {
  toastEl.innerHTML = html; toastEl.classList.add('show');
  clearTimeout(toastTimer); toastTimer = setTimeout(() => toastEl.classList.remove('show'), ms);
}

function setTarget(t, fromUser = false) {
  const res = sys.setTarget(t);
  const shown = res === 'queued' ? sys.pendingRef : sys.ref;
  $('tx').value = +shown[0].toFixed(1); $('ty').value = +shown[1].toFixed(1); $('tz').value = +shown[2].toFixed(1);
  if (res === 'queued' && fromUser) toast('Returning to base. Target <b>queued</b> for after the battery swap.');
  if (fromUser && $('tour').checked) { $('tour').checked = false; clearTimeout(tourTimer); tourTimer = null; }
}
function randomTarget() {
  const cur = sys.drone.p; let t;
  do { t = [rand(-16, 16), rand(-16, 16), rand(2.5, 14)]; } while (Math.hypot(t[0] - cur[0], t[1] - cur[1], t[2] - cur[2]) < 10);
  setTarget(t.map(v => Math.round(v * 2) / 2));
}

function boot() {
  sys = new FlightSystem();
  wireSystem();
  applyControls();
  setTarget([+$('tx').value, +$('ty').value, +$('tz').value]);
  renderModel({ Xi: sys.mpc.Xi, version: sys.modelVersion, reason: 'calibration' }, false);
  renderTruth(); drawChart();
  $('loading').classList.add('done');
  toast('Calibration flight done → <b>model v1</b> identified with SINDy. Tap the ground to set a target.', 5000);
}

function wireSystem() {
  sys.on('model', (m) => renderModel(m, true));
  sys.on('flight', (f) => {
    const errTxt = (v) => v.toFixed(2);
    let msg = `<b>Flight ${f.flight}</b> logged · `;
    if (!f.adaptive) msg += `model frozen at v${sys.modelVersion} · prediction error ${errTxt(f.before)} m/s²`;
    else if (f.accepted) msg += `SINDy refit → <b>model v${sys.modelVersion}</b> · error ${errTxt(f.before)} → ${errTxt(f.after)} m/s²`;
    else msg += `refit kept out: it didn't beat v${sys.modelVersion} on held-out data`;
    toast(msg); renderTruth(); drawChart();
    $('hudFlights').textContent = f.flight;
  });
  sys.on('mode', (m) => {
    const label = { mission: 'Mission', rtb: 'Return to base', landing: 'Landing', charging: 'Swapping battery' }[m.mode];
    $('hudMode').textContent = label; $('hudMode').className = `v mode ${m.mode}`;
    if (m.mode === 'rtb') toast(`Battery at <b>${Math.round(m.battery * 100)}%</b>: returning to base.`);
    if (m.mode === 'charging') setTimeout(() => toast('Landed on the pad. <b>Swapping battery</b>…'), 1200);
    if (m.mode === 'mission' && m.swapped) toast(m.resumed ? 'Fresh battery. <b>Resuming</b> the mission.' : 'Fresh battery. Hovering over the pad, ready for a target.');
  });
  sys.on('reset', () => { trailN = 0; renderTruth(); drawChart(); $('hudFlights').textContent = 0; });
}

function applyControls() {
  sys.wind.speed = +$('windSpeed').value; sys.wind.dir = +$('windDir').value; sys.wind.gust = +$('gust').value;
  sys.adaptive = $('adaptive').checked; sys.offsetFree = $('offsetFree').checked; sys.mpc.offsetFree = sys.offsetFree;
  if (!sys.offsetFree) sys.mpc.d.fill(0);
  sys.driftRate = +$('drift').value;
  sys.rtbLevel = +$('rtb').value / 100; $('rtbOut').textContent = `${$('rtb').value}%`;
  $('rtbMark').style.left = `${$('rtb').value}%`;
  $('windSpeedOut').textContent = `${sys.wind.speed} m/s`;
  $('windDirOut').textContent = `${sys.wind.dir}° ${compass(sys.wind.dir)}`;
  $('gustOut').textContent = sys.wind.gust.toFixed(2);
  $('driftOut').textContent = `${sys.driftRate.toFixed(2).replace(/0$/, '')}×`;
}
const compass = (d) => ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'][Math.round(d / 45) % 8];
for (const id of ['windSpeed', 'windDir', 'gust', 'drift', 'adaptive', 'offsetFree', 'rtb']) $(id).addEventListener('input', applyControls);
$('adaptive').addEventListener('change', (e) => toast(e.target.checked ? 'Refitting <b>on</b>: SINDy runs on every flight log.' : 'Refitting <b>off</b>: the model is frozen while the drone keeps ageing.'));
$('offsetFree').addEventListener('change', (e) => toast(e.target.checked ? 'Disturbance estimator <b>on</b>.' : 'Disturbance estimator <b>off</b>: the MPC now relies purely on its identified model.'));
$('reset').addEventListener('click', () => {
  $('loading').classList.remove('done');
  setTimeout(() => { sys.resetDrone(); applyControls(); setTarget(sys.ref); prevP = [0, 0, 0]; $('loading').classList.add('done'); toast('New drone, fresh calibration → <b>model v1</b>'); }, 30);
});
$('targetForm').addEventListener('submit', (e) => {
  e.preventDefault();
  setTarget([+$('tx').value || 0, +$('ty').value || 0, +$('tz').value || 5], true);
  document.activeElement?.blur();
});
$('tour').addEventListener('change', (e) => { if (e.target.checked && sys.flightDone) randomTarget(); });
document.querySelectorAll('.speed button').forEach((b) => b.addEventListener('click', () => {
  speed = +b.dataset.speed; document.querySelectorAll('.speed button').forEach((x) => x.classList.toggle('on', x === b));
}));

// tap / click on the ground → new target (drags still orbit)
const ray = new THREE.Raycaster(), ndc = new THREE.Vector2(); let down = null;
canvas.addEventListener('pointerdown', (e) => { down = { x: e.clientX, y: e.clientY, t: performance.now() }; });
canvas.addEventListener('pointerup', (e) => {
  if (!down || !sys) return;
  const moved = Math.hypot(e.clientX - down.x, e.clientY - down.y), dt = performance.now() - down.t; down = null;
  if (moved > 8 || dt > 600) return;
  const r = canvas.getBoundingClientRect();
  ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  ray.setFromCamera(ndc, camera);
  const hit = ray.intersectObject(ground)[0];
  if (!hit) return;
  setTarget([Math.round(hit.point.x * 2) / 2, Math.round(-hit.point.z * 2) / 2, +$('tz').value || 6], true);
  $('hint').textContent = `Target set to (${sys.ref.map((v) => v.toFixed(1)).join(', ')}). Change Z in the panel.`;
});

// ======================================================================== live panels
const COLS = [[0, '1'], [4, 'vx'], [5, 'vy'], [6, 'vz'], [7, 'throttle'], [8, 'roll'], [9, 'pitch']];
const ROWS = [[3, 'v̇x'], [4, 'v̇y'], [5, 'v̇z']];
let prevXi = null;
function renderModel(m, flash) {
  const t = $('coefTable');
  if (!t.tHead) {
    t.innerHTML = `<thead><tr class="group-head"><th></th><th colspan="4">bias · velocity</th><th colspan="3">inputs</th></tr>
      <tr><th></th>${COLS.map(([, n]) => `<th>${n}</th>`).join('')}</tr></thead><tbody>${ROWS.map(([, n]) =>
      `<tr><td>${n}</td>${COLS.map(() => '<td></td>').join('')}</tr>`).join('')}</tbody>`;
  }
  ROWS.forEach(([ri], r) => COLS.forEach(([ci], c) => {
    const td = t.tBodies[0].rows[r].cells[c + 1], v = m.Xi[ri][ci];
    td.textContent = Math.abs(v) < 1e-9 ? '·' : v.toFixed(3);
    td.className = Math.abs(v) < 1e-9 ? 'zero' : v > 0 ? 'pos' : 'neg';
    if (flash && prevXi && Math.abs(prevXi[ri][ci] - v) > 0.004) { td.classList.add('flash'); requestAnimationFrame(() => setTimeout(() => td.classList.remove('flash'), 60)); }
  }));
  prevXi = m.Xi.map((r) => Array.from(r));
  $('coefVersion').textContent = `model v${m.version}`;
  $('hudModel').textContent = `v${m.version}`;
  renderTruth();
}
function renderTruth() {
  if (!sys) return;
  const w = sys.drone.wear, Xi = sys.mpc.Xi, tf = sys.drone.thrustFactor();
  $('truHover').textContent = `${(50 / tf).toFixed(1)}%`;
  $('truGain').textContent = `${(9.81 / 50 * tf).toFixed(3)} m/s²`;
  $('modGain').textContent = `${Xi[5][7].toFixed(3)} m/s²`;
  $('modHover').textContent = `${Math.min(100, -Xi[5][0] / Xi[5][7]).toFixed(1)}%`;
  $('truBias').textContent = `${w.rollBias.toFixed(1)}° / ${w.pitchBias.toFixed(1)}°`;
}
function health(id, valId, frac, text, warnAt, badAt) {
  const el = $(id); el.style.width = `${Math.max(0, Math.min(1, frac)) * 100}%`;
  el.className = frac <= badAt ? 'bad' : frac <= warnAt ? 'warn' : '';
  $(valId).textContent = text;
}
function renderHealth() {
  const d = sys.drone, w = d.wear, bias = Math.hypot(w.rollBias, w.pitchBias), tw = d.thrustToWeight();
  health('barBatt', 'valBatt', d.battery, `${Math.round(d.battery * 100)}%`, 0.5, sys.rtbLevel);
  health('barMotor', 'valMotor', w.thrust, `${Math.round(w.thrust * 100)}%`, 0.85, 0.75);
  health('barImu', 'valImu', 1 - bias / 8, `${bias.toFixed(1)}°`, 0.6, 0.3);
  health('barTW', 'valTW', tw / 2, `${tw.toFixed(2)}×`, 0.75, 0.62);
  $('hudBatt').textContent = `${Math.round(d.battery * 100)}%`;
}
function drawChart() {
  const c = $('errChart'), g = c.getContext('2d'), dpr = Math.min(window.devicePixelRatio, 2);
  const W = c.clientWidth, H = c.clientHeight; c.width = W * dpr; c.height = H * dpr; g.scale(dpr, dpr);
  g.clearRect(0, 0, W, H);
  const hist = sys ? sys.history : [];
  const muted = css('--muted'), border = css('--border');
  const padL = 34, padB = 20, padT = 6, iw = W - padL - 4, ih = H - padB - padT;
  const ymax = Math.max(2, ...hist.map((h) => h.before)) * 1.1;
  g.font = '11px IBM Plex Mono, monospace'; g.fillStyle = muted; g.strokeStyle = border; g.lineWidth = 1;
  for (const v of [0, ymax / 2, ymax]) {
    const y = padT + ih - (v / ymax) * ih;
    g.beginPath(); g.moveTo(padL, Math.round(y) + 0.5); g.lineTo(W - 4, Math.round(y) + 0.5); g.stroke();
    g.textAlign = 'right'; g.fillText(v.toFixed(1), padL - 6, y + 4);
  }
  if (!hist.length) { g.textAlign = 'center'; g.fillText('Waiting for the first flight to finish…', padL + iw / 2, padT + ih / 2); return; }
  const n = Math.max(hist.length, 10), bw = iw / n, gap = Math.min(4, bw * 0.25);
  hist.forEach((h, i) => {
    const bh = (h.before / ymax) * ih, x = padL + i * bw + gap / 2;
    g.fillStyle = h.adaptive ? css('--adapt') : css('--frozen');
    const r = Math.min(3, (bw - gap) / 2);
    g.beginPath(); g.roundRect(x, padT + ih - bh, bw - gap, bh, [r, r, 0, 0]); g.fill();
    if (hist.length <= 14 || i % Math.ceil(hist.length / 10) === 0) { g.fillStyle = muted; g.textAlign = 'center'; g.fillText(h.flight, x + (bw - gap) / 2, H - 5); }
  });
}
new ResizeObserver(() => sys && drawChart()).observe($('errChart'));
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => sys && drawChart());

function setBar(id, valId, v, centered, max, unit) {
  const el = $(id);
  if (!centered) { el.style.left = '0'; el.style.width = `${Math.max(0, Math.min(100, v / max * 100))}%`; }
  else { const f = Math.max(-1, Math.min(1, v / max)) * 50; el.style.left = `${f >= 0 ? 50 : 50 + f}%`; el.style.width = `${Math.abs(f)}%`; }
  $(valId).textContent = `${v.toFixed(0)}${unit}`;
}

// ======================================================================== frame loop
const v3 = new THREE.Vector3(), v3b = new THREE.Vector3();
let lastFrame = performance.now(), uiTick = 0;
new IntersectionObserver(([e]) => { visible = e.isIntersecting; }, { threshold: 0.05 }).observe(viewport);

function frame(now) {
  requestAnimationFrame(frame);
  const dtReal = Math.min(0.1, (now - lastFrame) / 1000); lastFrame = now;
  if (!sys || !visible) return;

  acc += dtReal * speed;
  let steps = 0;
  while (acc >= DT && steps < 8) {
    prevP = [...sys.drone.p]; prevAtt = [...sys.drone.att];
    last = sys.tick(); acc -= DT; steps++;
    if (last && trailN < TRAIL_MAX) {
      toThree(sys.drone.p, v3); trailGeo.attributes.position.setXYZ(trailN++, v3.x, v3.y, v3.z);
    } else if (last) {
      const a = trailGeo.attributes.position.array; a.copyWithin(0, 3); toThree(sys.drone.p, v3); a.set([v3.x, v3.y, v3.z], (TRAIL_MAX - 1) * 3);
    }
    if (sys.flightDone && sys.mode === 'mission' && $('tour').checked && !tourTimer) tourTimer = setTimeout(() => { tourTimer = null; if ($('tour').checked) randomTarget(); }, 1400 / speed);
  }
  if (steps) { trailGeo.setDrawRange(0, trailN); trailGeo.attributes.position.needsUpdate = true; }
  const alpha = Math.min(1, acc / DT);

  // drone pose (interpolated between control ticks)
  const p = prevP.map((v, i) => v + (sys.drone.p[i] - v) * alpha);
  const att = prevAtt.map((v, i) => v + (sys.drone.att[i] - v) * alpha);
  toThree(p, drone.position);
  tilt.rotation.set(att[0] * Math.PI / 180, 0, -att[1] * Math.PI / 180, 'XZY');
  const spin = 0.25 + (last ? last.u[0] / U_MAX[0] : 0) * 1.4;
  rotors.forEach((r, i) => { r.rotation.y += spin * (i % 2 ? 1 : -1) * dtReal * 60 * 0.35; });

  shadow.position.set(drone.position.x, 0.02, drone.position.z);
  const sc = 1.6 + p[2] * 0.06; shadow.scale.set(sc, sc, 1); shadow.material.opacity = Math.max(0.1, 0.5 - p[2] * 0.02);
  plumbGeo.attributes.position.setXYZ(0, drone.position.x, drone.position.y, drone.position.z);
  plumbGeo.attributes.position.setXYZ(1, drone.position.x, 0, drone.position.z);
  plumbGeo.attributes.position.needsUpdate = true; plumb.computeLineDistances();

  // target
  toThree(sys.ref, target.position);
  targetHalo.scale.setScalar(1 + 0.15 * Math.sin(now / 300));
  targetRing.position.set(target.position.x, 0.03, target.position.z);
  stemGeo.attributes.position.setXYZ(0, target.position.x, target.position.y, target.position.z);
  stemGeo.attributes.position.setXYZ(1, target.position.x, 0, target.position.z);
  stemGeo.attributes.position.needsUpdate = true; stem.computeLineDistances();

  // MPC plan
  if (last) {
    const arr = predGeo.attributes.position;
    arr.setXYZ(0, drone.position.x, drone.position.y, drone.position.z);
    last.traj.forEach((q, i) => { toThree(q, v3); arr.setXYZ(i + 1, v3.x, v3.y, v3.z); });
    predGeo.setDrawRange(0, last.traj.length + 1); arr.needsUpdate = true;
  }

  // wind field: mean wind + the drone's current gust, streaks drift with it
  const w = last ? last.wind : [0, 0, 0];
  const pos = windGeo.attributes.position.array, len = 0.28;
  for (let i = 0; i < NP; i++) {
    const q = windPts[i], s = 0.7 + 0.6 * q[3];
    for (let k = 0; k < 3; k++) q[k] += w[k] * s * dtReal * speed;
    if (q[0] > half + 4) q[0] -= 2 * half + 8; if (q[0] < -half - 4) q[0] += 2 * half + 8;
    if (q[1] > half + 4) q[1] -= 2 * half + 8; if (q[1] < -half - 4) q[1] += 2 * half + 8;
    if (q[2] > ARENA.zMax + 2) q[2] = 0.3; if (q[2] < 0.3) q[2] = ARENA.zMax + 2;
    toThree(q, v3); toThree([q[0] - w[0] * len * s, q[1] - w[1] * len * s, q[2] - w[2] * len * s], v3b);
    pos.set([v3.x, v3.y, v3.z, v3b.x, v3b.y, v3b.z], i * 6);
  }
  windGeo.attributes.position.needsUpdate = true;
  const wmag = Math.hypot(...w);
  if (wmag > 0.05) {
    windArrow.visible = true; toThree(w, v3).normalize(); windArrow.setDirection(v3);
    windArrow.position.copy(drone.position).add(v3b.set(0, 2.2, 0)).addScaledVector(v3, -wmag * 0.25);
    windArrow.setLength(0.6 + wmag * 0.45, 0.5, 0.35);
  } else windArrow.visible = false;

  const blink = (sys.mode === 'rtb' || sys.mode === 'landing') && Math.floor(now / 250) % 2;
  led.material.color.setHex(sys.mode === 'charging' ? 0x2a323d : blink ? 0xff5a3c : sys.mode === 'mission' ? 0x38c6f0 : 0xf5a524);
  controls.update();
  renderer.render(scene, camera);

  // throttled DOM updates
  if (last && (uiTick++ % 4 === 0)) {
    $('hudErr').textContent = `${last.err.toFixed(2)} m`;
    $('hudWind').textContent = `${wmag.toFixed(1)} m/s ${compass((Math.atan2(w[0], w[1]) * 180 / Math.PI + 360) % 360)}`;
    setBar('barT', 'valT', last.u[0], false, 100, '%');
    setBar('barR', 'valR', last.u[1], true, 30, '°');
    setBar('barP', 'valP', last.u[2], true, 30, '°');
    renderHealth();
  }
}

// boot after first paint so the loading overlay shows during the calibration flight
requestAnimationFrame(() => setTimeout(() => { boot(); requestAnimationFrame(frame); }, 30));
