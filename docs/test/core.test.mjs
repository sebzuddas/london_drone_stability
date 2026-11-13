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
    // 45 s of mission time per waypoint; time spent returning to base / swapping the battery doesn't count
    for (let k = 0, mission = 0; k < 1200 && mission < 450 && !sys.flightDone; k++) { sys.tick(); if (sys.mode === 'mission') mission++; }
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

// --- 4. low battery → return to base, land, swap, resume the queued target
{
  const sys = new FlightSystem();
  sys.wind = { speed: 4, dir: 70, gust: 0.5 };
  const modes = []; let landedAt = null, minBattery = 1, flightsAtLanding = 0;
  sys.on('mode', m => {
    modes.push(m.mode);
    if (m.mode === 'charging') { landedAt = [...sys.drone.p]; flightsAtLanding = sys.drone.wear.flights; }
  });
  sys.setTarget([15, 12, 8]);
  sys.drone.battery = 0.3;                       // start low so RTB comes quickly
  let queued = null;
  for (let k = 0; k < 2000 && !(modes.includes('mission') && modes.at(-1) === 'mission'); k++) {
    const o = sys.tick(); minBattery = Math.min(minBattery, o.battery);
    if (sys.mode === 'rtb' && !queued) queued = sys.setTarget([-10, 8, 6]);   // user taps while away
  }
  console.log(`\nRTB sequence: ${modes.join(' → ')}; landed at (${landedAt?.map(v => v.toFixed(2)).join(', ')}), min battery ${(minBattery * 100).toFixed(0)}%`);
  check(modes.join(',') === 'rtb,landing,charging,mission', 'battery below threshold triggers RTB → landing → swap → mission');
  check(landedAt && Math.hypot(landedAt[0], landedAt[1]) < 1.0 && landedAt[2] < 0.05, 'drone lands within 1 m of the home pad');
  check(sys.drone.battery === 1 && minBattery > 0.05, 'battery swapped without running flat');
  check(flightsAtLanding >= 1, 'landing ends a flight (so SINDy refits)');
  check(queued === 'queued' && sys.ref.join() === '-10,8,6', 'target set during RTB is queued and flown after the swap');
}

if (failures.length) { console.error(`\n${failures.length} check(s) failed`); process.exit(1); }
console.log('\nall checks passed');
