// Headless check of the demo's numerical core (no browser needed):
//   node docs/test/core.test.mjs
// 1. the calibration flight identifies gains close to the hidden plant physics
// 2. the MPC reaches a series of waypoints in gusty wind while the drone ages
// 3. refitting after each flight keeps model error low; a frozen model's error grows with drift
import { FlightSystem, LIB_NAMES, STATE_NAMES } from '../js/core.js';

const fmt = v => (Math.abs(v) < 1e-9 ? '    ·  ' : v.toFixed(3).padStart(7));
const failures = [];
const check = (cond, msg) => { console.log(`${cond ? 'PASS' : 'FAIL'}  ${msg}`); if (!cond) failures.push(msg); };

// --- 1. identification
{
  const sys = new FlightSystem();
  const Xi = sys.mpc.Xi;
  console.log('Calibration model (rows = ẋ, cols = library):\n        ' + LIB_NAMES.map(n => n.padStart(7)).join(''));
  Xi.forEach((r, i) => console.log(('d' + STATE_NAMES[i]).padEnd(8) + Array.from(r, fmt).join('')));
  const near = (v, truth, tol) => Math.abs(v - truth) <= tol * Math.abs(truth);
  check(near(Xi[5][7], 9.81 / 50, 0.15), `throttle gain ${Xi[5][7].toFixed(3)} ≈ 0.196`);
  check(near(Xi[3][9], 9.81 * Math.PI / 180, 0.15), `pitch gain ${Xi[3][9].toFixed(3)} ≈ 0.171`);
  check(near(Xi[4][8], -9.81 * Math.PI / 180, 0.15), `roll gain ${Xi[4][8].toFixed(3)} ≈ -0.171`);
  check(near(Xi[5][0], -9.81, 0.1), `gravity term ${Xi[5][0].toFixed(2)} ≈ -9.81`);
}

// --- 2 & 3. closed loop, adaptive vs frozen
const targets = [[10, 5, 8], [-12, 10, 4], [0, -15, 12], [15, 15, 3], [-5, -5, 10], [8, -12, 6], [0, 0, 5], [12, 3, 14]];
const run = adaptive => {
  const sys = new FlightSystem();
  sys.adaptive = adaptive; sys.driftRate = 1.5; sys.wind = { speed: 5, dir: 70, gust: 0.6 };
  const errs = []; let settled = 0;
  sys.on('flight', f => errs.push(f.before));
  for (const t of targets) {
    sys.setTarget(t);
    for (let k = 0; k < 450 && !sys.flightDone; k++) sys.tick();
    if (sys.flightDone) settled++;
  }
  return { settled, errs };
};
const adaptive = run(true), frozen = run(false);
const tail = a => a.slice(-3).reduce((s, v) => s + v, 0) / 3;
console.log(`\nadaptive: settled ${adaptive.settled}/${targets.length}, model error per flight ${adaptive.errs.map(v => v.toFixed(2)).join(' ')}`);
console.log(`frozen:   settled ${frozen.settled}/${targets.length}, model error per flight ${frozen.errs.map(v => v.toFixed(2)).join(' ')}`);
check(adaptive.settled === targets.length, 'adaptive MPC settles on every waypoint in gusty wind');
check(tail(adaptive.errs) < 0.7 * tail(frozen.errs), 'refitting keeps model error well below the frozen model after drift');

if (failures.length) { console.error(`\n${failures.length} check(s) failed`); process.exit(1); }
console.log('\nall checks passed');
