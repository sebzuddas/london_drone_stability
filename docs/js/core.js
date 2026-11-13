// Numerical core for the browser demo: plant + wind, SINDy (STLSQ), and a condensed linear MPC.
// Pure JS, no DOM — imported by the page and by the headless test in docs/test/.

export const DT = 0.1;            // control / logging period (s), same 100 ms as the hackathon server
const SUBSTEPS = 10;               // physics integration steps per control period
const G = 9.81;
export const ARENA = { half: 20, zMax: 18 };
export const U_MIN = [0, -30, -30];   // throttle %, roll deg, pitch deg
export const U_MAX = [100, 30, 30];
export const STATE_NAMES = ['x', 'y', 'z', 'vx', 'vy', 'vz'];
export const LIB_NAMES = ['1', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'throttle', 'roll', 'pitch'];

// ---------------------------------------------------------------- small dense linear algebra
export const zeros = (r, c) => Array.from({ length: r }, () => new Float64Array(c));
export const eye = (n) => { const M = zeros(n, n); for (let i = 0; i < n; i++) M[i][i] = 1; return M; };
export function matmul(A, B) {
  const r = A.length, k = B.length, c = B[0].length, C = zeros(r, c);
  for (let i = 0; i < r; i++) for (let p = 0; p < k; p++) {
    const a = A[i][p]; if (a === 0) continue;
    const Bp = B[p], Ci = C[i];
    for (let j = 0; j < c; j++) Ci[j] += a * Bp[j];
  }
  return C;
}
export const matvec = (A, x) => A.map(row => { let s = 0; for (let j = 0; j < x.length; j++) s += row[j] * x[j]; return s; });
// Solve symmetric positive (semi)definite system with Gaussian elimination + partial pivoting.
export function solve(Ain, bin) {
  const n = Ain.length, A = Ain.map(r => Float64Array.from(r)), b = Float64Array.from(bin);
  for (let i = 0; i < n; i++) {
    let p = i; for (let r = i + 1; r < n; r++) if (Math.abs(A[r][i]) > Math.abs(A[p][i])) p = r;
    [A[i], A[p]] = [A[p], A[i]]; [b[i], b[p]] = [b[p], b[i]];
    const piv = A[i][i]; if (Math.abs(piv) < 1e-14) continue;
    for (let r = i + 1; r < n; r++) {
      const f = A[r][i] / piv; if (f === 0) continue;
      for (let c = i; c < n; c++) A[r][c] -= f * A[i][c];
      b[r] -= f * b[i];
    }
  }
  const x = new Float64Array(n);
  for (let i = n - 1; i >= 0; i--) {
    let s = b[i]; for (let c = i + 1; c < n; c++) s -= A[i][c] * x[c];
    x[i] = Math.abs(A[i][i]) < 1e-14 ? 0 : s / A[i][i];
  }
  return x;
}
// Matrix exponential, scaling-and-squaring with a Taylor series.
export function expm(M) {
  const n = M.length; let norm = 0;
  for (const row of M) norm = Math.max(norm, row.reduce((s, v) => s + Math.abs(v), 0));
  const s = Math.max(0, Math.ceil(Math.log2(norm || 1)) + 1);
  const Ms = M.map(r => r.map(v => v / 2 ** s));
  let E = eye(n), term = eye(n);
  for (let k = 1; k <= 14; k++) {
    term = matmul(term, Ms).map(r => r.map(v => v / k));
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) E[i][j] += term[i][j];
  }
  for (let i = 0; i < s; i++) E = matmul(E, E);
  return E;
}

// ---------------------------------------------------------------- random helpers
export function gauss() {
  let u = 0, v = 0; while (u === 0) u = Math.random(); while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

// ---------------------------------------------------------------- plant: quadrotor + wind + long-term drift
// The "real" drone. The controller never sees these equations — it only sees logged positions.
export class Drone {
  constructor() { this.reset(true); }
  reset(resetWear = false) {
    this.p = [0, 0, 0]; this.v = [0, 0, 0];
    this.att = [0, 0];                   // actual roll, pitch (deg)
    this.throttle = 0;
    if (resetWear) this.wear = { thrust: 1, rollBias: 0, pitchBias: 0, flights: 0 };
    this.windGust = [0, 0, 0];
    this.battery = 1;                    // state of charge, 0..1 (swapped on the pad)
  }
  // Thrust available relative to a new drone on a full battery: motor wear × battery voltage sag.
  thrustFactor() { return this.wear.thrust * (0.92 + 0.08 * this.battery); }
  // Thrust-to-weight ratio at 100% throttle. Below ~1.15 the drone can no longer climb while manoeuvring.
  thrustToWeight() { return 2 * this.thrustFactor(); }
  // One flight's worth of ageing: motors/battery lose thrust, the IMU's level reference creeps.
  age(rate) {
    const w = this.wear; w.flights++;
    // Wear levels off instead of compounding forever: thrust decays towards 72% (hover ≈ 70% throttle, still flyable
    // when tilted), IMU bias towards about +6° / -4°. Each flight closes a fraction of the remaining gap.
    const k = Math.min(1, 0.12 * rate);
    w.thrust = 0.72 + (w.thrust - 0.72) * (1 - k);
    w.rollBias += (6 - w.rollBias) * k; w.pitchBias += (-4 - w.pitchBias) * k;
  }
  wind(windCfg) {
    const rad = windCfg.dir * Math.PI / 180;
    // windCfg.dir is the compass direction the wind blows *towards*
    return [Math.sin(rad) * windCfg.speed + this.windGust[0],
            Math.cos(rad) * windCfg.speed + this.windGust[1],
            this.windGust[2]];
  }
  step(u, windCfg) {
    const h = DT / SUBSTEPS, w = this.wear;
    const tauAtt = 0.12, drag = 0.35, kT = G / 50;
    for (let s = 0; s < SUBSTEPS; s++) {
      // gusts: Ornstein–Uhlenbeck per axis
      const sig = windCfg.gust * (1.5 + 0.4 * windCfg.speed), tau = 1.6;
      for (let i = 0; i < 3; i++) {
        const sc = i === 2 ? 0.35 : 1;
        this.windGust[i] += -this.windGust[i] / tau * h + sc * sig * Math.sqrt(2 * h / tau) * gauss();
      }
      const wv = this.wind(windCfg);
      this.att[0] += ((u[1] + w.rollBias) - this.att[0]) / tauAtt * h;
      this.att[1] += ((u[2] + w.pitchBias) - this.att[1]) / tauAtt * h;
      this.throttle += (u[0] - this.throttle) / 0.05 * h;
      const phi = this.att[0] * Math.PI / 180, th = this.att[1] * Math.PI / 180;
      const a = kT * this.thrustFactor() * Math.max(0, this.throttle);
      // battery drain grows with throttle (≈1%/s in hover); it keeps a small reserve so sag never hits zero thrust
      if (this.throttle > 1) this.battery = Math.max(0, this.battery - h * (0.003 + 0.017 * (this.throttle / 100) ** 1.5));
      const zb = [Math.sin(th) * Math.cos(phi), -Math.sin(phi), Math.cos(th) * Math.cos(phi)];
      const acc = [0, 1, 2].map(i => a * zb[i] - (i === 2 ? G : 0) - drag * (this.v[i] - wv[i]));
      for (let i = 0; i < 3; i++) { this.v[i] += acc[i] * h; this.p[i] += this.v[i] * h; }
      if (this.p[2] <= 0) { this.p[2] = 0; if (this.v[2] < 0) this.v[2] = 0; this.v[0] *= 0.9; this.v[1] *= 0.9; }
    }
  }
  measure(noise = 0.01) { return this.p.map(v => v + noise * gauss()); }
}

// ---------------------------------------------------------------- velocity estimator (positions in, [p, v] out)
export class Estimator {
  constructor() { this.x = null; }
  update(pm) {
    if (!this.x) { this.x = [...pm, 0, 0, 0]; return this.x; }
    const a = 0.55, b = 0.35;            // alpha–beta filter
    for (let i = 0; i < 3; i++) {
      const pPred = this.x[i] + DT * this.x[3 + i];
      const r = pm[i] - pPred;
      this.x[i] = pPred + a * r;
      this.x[3 + i] += (b / DT) * r;
    }
    return this.x;
  }
}

// ---------------------------------------------------------------- SINDy with control: ẋ = Ξᵀ θ(x, u)
// Library = constant + identity on states + identity on inputs, optimiser = STLSQ (as in pysindy).
// The logs only hold measured positions and inputs; velocities/accelerations come from smoothed finite differences.
function smooth(series, half = 2) {
  return series.map((_, k) => {
    let s = 0, n = 0;
    for (let j = Math.max(0, k - half); j <= Math.min(series.length - 1, k + half); j++) { s += series[j]; n++; }
    return s / n;
  });
}
const centralDiff = (s) => s.map((_, k) => {
  const a = Math.max(0, k - 1), b = Math.min(s.length - 1, k + 1);
  return (s[b] - s[a]) / ((b - a) * DT);
});

export function buildRegression(log, lag = LAG) {
  // log: array of {p:[x,y,z], u:[T,roll,pitch]} sampled every DT.
  // Every signal goes through the *same* low-pass filter so regressors and derivative targets stay consistent
  // (a linear filter commutes with a linear model; filtering only the targets would bias the fit).
  const filt = s => smooth(smooth(s, 3), 3);
  const pos = [0, 1, 2].map(i => filt(log.map(r => r.p[i])));
  const inp = [0, 1, 2].map(c => filt(log.map(r => r.u[c])));
  const vel = pos.map(centralDiff);
  const acc = vel.map(centralDiff);
  const rows = [], targets = [];
  for (let k = 8; k < log.length - 8; k++) {
    // actuators respond with a short delay (ZOH + attitude/motor lag), so pair ẋ(k) with u(k - lag)
    if (log[k].p[2] < 0.3) continue;               // ground contact isn't part of the model — skip it
    const u = inp.map(s => s[k - lag]);
    const x = [pos[0][k], pos[1][k], pos[2][k], vel[0][k], vel[1][k], vel[2][k]];
    rows.push([1, ...x, ...u]);
    targets.push([vel[0][k], vel[1][k], vel[2][k], acc[0][k], acc[1][k], acc[2][k]]);
  }
  return { rows, targets };
}
export const LAG = 1;
export let PRIOR_WEIGHT = 0.5;
export const setPriorWeight = w => { PRIOR_WEIGHT = w; };

// prior (optional): previous Ξ. With it, the ridge pulls towards the old model instead of towards zero, so
// directions a short closed-loop flight barely excites (e.g. throttle vs. the bias term while hovering) stay put.
// Physics-informed coefficient mask (constrained SINDy): which library terms each ẋ row may use.
// Translation invariance → no position terms; aerodynamic drag acts on an axis' own velocity; inputs may act anywhere.
//   library: [1, x, y, z, vx, vy, vz, throttle, roll, pitch]
export const MASK = [
  [0, 0, 0, 0, 1, 1, 1, 0, 0, 0],  // ẋ  = f(v)
  [0, 0, 0, 0, 1, 1, 1, 0, 0, 0],  // ẏ
  [0, 0, 0, 0, 1, 1, 1, 0, 0, 0],  // ż
  [1, 0, 0, 0, 1, 0, 0, 1, 1, 1],  // v̇x
  [1, 0, 0, 0, 0, 1, 0, 1, 1, 1],  // v̇y
  [1, 0, 0, 0, 0, 0, 1, 1, 1, 1],  // v̇z
];
export function stlsq(rows, targets, { threshold = 0.05, alpha = 0.05, iters = 10, prior = null, priorWeight = PRIOR_WEIGHT, mask = MASK } = {}) {
  const nf = rows[0].length, nt = targets[0].length;
  // column scaling so the ridge penalty treats features fairly; threshold applies to unscaled coefs
  const scale = Array.from({ length: nf }, (_, j) => Math.sqrt(rows.reduce((s, r) => s + r[j] * r[j], 0) / rows.length) || 1);
  const Xs = rows.map(r => r.map((v, j) => v / scale[j]));
  const Xi = zeros(nt, nf);
  for (let t = 0; t < nt; t++) {
    let active = Array.from({ length: nf }, (_, j) => j).filter(j => !mask || mask[t][j]);
    let coef = new Float64Array(nf);
    for (let it = 0; it < iters; it++) {
      const m = active.length, AtA = zeros(m, m), Atb = new Float64Array(m);
      for (let r = 0; r < Xs.length; r++) {
        const row = Xs[r], y = targets[r][t];
        for (let a = 0; a < m; a++) {
          const va = row[active[a]]; Atb[a] += va * y;
          for (let b = a; b < m; b++) AtA[a][b] += va * row[active[b]];
        }
      }
      const lam = prior ? priorWeight * Xs.length : alpha;
      for (let a = 0; a < m; a++) {
        for (let b = 0; b < a; b++) AtA[a][b] = AtA[b][a];
        AtA[a][a] += lam;
        if (prior) Atb[a] += lam * prior[t][active[a]] * scale[active[a]];
      }
      const sol = solve(AtA, Atb);
      coef = new Float64Array(nf); active.forEach((j, a) => { coef[j] = sol[a] / scale[j]; });
      const next = active.filter(j => (j === 0 && mask[t][0]) || Math.abs(coef[j]) >= threshold);  // never prune the bias column
      if (next.length === active.length) break;
      active = next;
    }
    Xi[t] = coef;
  }
  return Xi; // nt × nf, rows = ẋ components, cols = library terms
}

export function fitSindy(log, opts) {
  const { rows, targets } = Array.isArray(log[0]) ? mergeRegressions(log) : buildRegression(log);
  if (rows.length < 30) return null;
  const Xi = stlsq(rows, targets, opts);
  return { Xi, rmse: predictionRmse(Xi, rows, targets) };
}

// RMSE of predicted acceleration (the hard rows) for a model on a log
export function predictionRmse(Xi, rows, targets) {
  let s = 0, n = 0;
  for (let r = 0; r < rows.length; r++) for (let t = 3; t < 6; t++) {
    let pred = 0; for (let j = 0; j < rows[r].length; j++) pred += Xi[t][j] * rows[r][j];
    s += (pred - targets[r][t]) ** 2; n++;
  }
  return Math.sqrt(s / n);
}
// several flight logs → one regression problem (each log differentiated separately so seams don't create spikes)
export function mergeRegressions(logs) {
  const rows = [], targets = [];
  for (const l of logs) { const r = buildRegression(l); rows.push(...r.rows); targets.push(...r.targets); }
  return { rows, targets };
}
export function modelErrorOnLog(Xi, log) {
  const { rows, targets } = buildRegression(log);
  return rows.length ? predictionRmse(Xi, rows, targets) : NaN;
}

// ---------------------------------------------------------------- linear MPC on the identified model
export class MPC {
  constructor({ N = 20 } = {}) {
    this.N = N; this.n = 6; this.m = 3;
    this.Q = [6, 6, 8, 0.6, 0.6, 0.8];
    this.Qf = this.Q.map(q => q * 6);
    this.R = [0.004, 0.02, 0.02];
    this.S = [0.004, 0.03, 0.03];          // input-rate penalty (smoothness)
    this.d = new Float64Array(6);          // offset-free disturbance estimate (wind etc.)
    this.offsetFree = true; this.obsGain = 0.05;
    this.U = null; this.uPrev = [50, 0, 0];
  }
  setModel(Xi) {
    // continuous affine model ẋ = A x + B u + c, discretised exactly with an augmented matrix exponential
    const n = this.n, m = this.m, M = zeros(n + m + 1, n + m + 1);
    for (let i = 0; i < n; i++) {
      M[i][n + m] = Xi[i][0];
      for (let j = 0; j < n; j++) M[i][j] = Xi[i][1 + j] * DT;
      for (let j = 0; j < m; j++) M[i][n + j] = Xi[i][1 + n + j] * DT;
      M[i][n + m] *= DT;
    }
    const E = expm(M);
    this.Ad = E.slice(0, n).map(r => Float64Array.from(r.slice(0, n)));
    this.Bd = E.slice(0, n).map(r => Float64Array.from(r.slice(n, n + m)));
    this.cd = E.slice(0, n).map(r => r[n + m]);
    this.Xi = Xi;
    this.buildQP();
    this.d.fill(0);
  }
  buildQP() {
    const { N, n, m, Ad, Bd } = this, nu = N * m;
    // Γ: stacked state response to the input sequence
    const AB = [Bd]; for (let i = 1; i < N; i++) AB.push(matmul(Ad, AB[i - 1]));
    this.Gam = zeros(N * n, nu);
    for (let i = 0; i < N; i++) for (let j = 0; j <= i; j++) {
      const blk = AB[i - j];
      for (let r = 0; r < n; r++) for (let c = 0; c < m; c++) this.Gam[i * n + r][j * m + c] = blk[r][c];
    }
    const qd = []; for (let i = 0; i < N; i++) qd.push(...(i === N - 1 ? this.Qf : this.Q));
    this.qd = qd;
    const H = zeros(nu, nu);
    for (let a = 0; a < nu; a++) for (let b = a; b < nu; b++) {
      let s = 0; for (let r = 0; r < N * n; r++) s += this.Gam[r][a] * qd[r] * this.Gam[r][b];
      H[a][b] = H[b][a] = s;
    }
    for (let i = 0; i < N; i++) for (let c = 0; c < m; c++) {
      const k = i * m + c; H[k][k] += this.R[c] + 2 * this.S[c];
      if (i === N - 1) H[k][k] -= this.S[c];
      if (i > 0) { H[k][k - m] -= this.S[c]; H[k - m][k] -= this.S[c]; }
    }
    this.H = H;
    // Lipschitz constant for the projected-gradient solver (power iteration)
    let v = new Float64Array(nu).fill(1), L = 1;
    for (let it = 0; it < 60; it++) {
      const w = matvec(H, v); L = Math.hypot(...w); v = Float64Array.from(w, x => x / L);
    }
    this.Lip = L * 1.05;
  }
  steadyInput(ref) {
    // input that holds the model still at the reference: velocity rows of A x + B u + c + d = 0
    const B = [3, 4, 5].map(i => [...this.Xi[i].slice(7, 10)]);
    const rhs = [3, 4, 5].map(i => {
      let s = this.Xi[i][0]; for (let j = 0; j < 3; j++) s += this.Xi[i][1 + j] * ref[j];
      return -(s + this.dCont(i));
    });
    const u = solve(B, rhs);
    return [0, 1, 2].map(c => Math.min(U_MAX[c], Math.max(U_MIN[c], Number.isFinite(u[c]) ? u[c] : this.uPrev[c])));
  }
  dCont(i) { return this.d[i] / DT; }
  observe(x, xPrev, uPrev) {
    // offset-free MPC: integrate one-step prediction error into a disturbance estimate (velocity rows)
    if (!this.offsetFree || !xPrev) return;
    const pred = matvec(this.Ad, xPrev).map((v, i) => v + matvec([this.Bd[i]], uPrev)[0] + this.cd[i] + this.d[i]);
    for (let i = 3; i < 6; i++) this.d[i] = Math.max(-1, Math.min(1, this.d[i] + this.obsGain * (x[i] - pred[i])));
  }
  solve(x, ref) {
    const { N, n, m } = this, nu = N * m;
    const uss = this.steadyInput(ref);
    // free response (U = 0) including bias and disturbance
    const free = new Float64Array(N * n); let xi = Float64Array.from(x);
    for (let i = 0; i < N; i++) {
      const nx = matvec(this.Ad, xi).map((v, r) => v + this.cd[r] + this.d[r]);
      for (let r = 0; r < n; r++) free[i * n + r] = nx[r];
      xi = Float64Array.from(nx);
    }
    const target = [...ref, 0, 0, 0];
    const err = Array.from(free, (v, k) => this.qd[k] * (v - target[k % n]));
    const f = new Float64Array(nu);
    for (let a = 0; a < nu; a++) { let s = 0; for (let r = 0; r < N * n; r++) s += this.Gam[r][a] * err[r]; f[a] = s; }
    for (let i = 0; i < N; i++) for (let c = 0; c < m; c++) f[i * m + c] -= this.R[c] * uss[c];
    for (let c = 0; c < m; c++) f[c] -= this.S[c] * this.uPrev[c];
    // FISTA projected gradient on ½UᵀHU + fᵀU, box constraints on every input
    const lo = i => U_MIN[i % m], hi = i => U_MAX[i % m];
    let U = new Float64Array(nu);
    if (this.U) { for (let k = 0; k < nu - m; k++) U[k] = this.U[k + m]; for (let c = 0; c < m; c++) U[nu - m + c] = this.U[nu - m + c]; }
    else for (let k = 0; k < nu; k++) U[k] = uss[k % m];
    let Y = Float64Array.from(U), t = 1;
    for (let it = 0; it < 120; it++) {
      const g = matvec(this.H, Y);
      const Un = new Float64Array(nu);
      for (let k = 0; k < nu; k++) Un[k] = Math.min(hi(k), Math.max(lo(k), Y[k] - (g[k] + f[k]) / this.Lip));
      const tn = (1 + Math.sqrt(1 + 4 * t * t)) / 2;
      for (let k = 0; k < nu; k++) Y[k] = Un[k] + ((t - 1) / tn) * (Un[k] - U[k]);
      U = Un; t = tn;
    }
    this.U = U;
    const u = [U[0], U[1], U[2]];
    this.uPrev = u;
    // predicted trajectory for display
    const traj = []; const XU = matvec(this.Gam, U);
    for (let i = 0; i < N; i++) traj.push([0, 1, 2].map(r => free[i * n + r] + XU[i * n + r]));
    return { u, traj, uss };
  }
}

// ---------------------------------------------------------------- simple PD hover controller for the calibration flight
export function calibrationPilot(x, ref, t) {
  const ex = ref.map((r, i) => r - x[i]);
  const T = 50 + 12 * ex[2] - 10 * x[5] + 8 * Math.sin(1.3 * t) + 6 * gauss();
  const pitch = 3 * ex[0] - 5 * x[3] + 8 * Math.sin(0.9 * t) + 4 * gauss();
  const roll = -(3 * ex[1] - 5 * x[4]) + 8 * Math.cos(0.7 * t) + 4 * gauss();
  return [T, roll, pitch].map((v, c) => Math.min(U_MAX[c], Math.max(U_MIN[c], v)));
}

// ---------------------------------------------------------------- whole-system controller that the UI drives
export class FlightSystem {
  constructor() {
    this.drone = new Drone(); this.est = new Estimator(); this.mpc = new MPC();
    this.wind = { speed: 4, dir: 70, gust: 0.5 };
    this.adaptive = true; this.driftRate = 1; this.dither = true;
    this.ref = [0, 0, 6];
    this.mode = 'mission'; this.home = [0, 0]; this.rtbLevel = 0.25; this.swapTime = 3;   // RTB + battery swap
    this.pendingRef = null; this.missionRef = null;
    this.flightLog = []; this.history = [];       // history: per-flight summaries
    this.window = [];                              // rolling window of recent flight logs used for refits
    this.modelVersion = 0; this.t = 0;
    this.listeners = {};
    this.calibrate();
  }
  on(ev, fn) { (this.listeners[ev] ||= []).push(fn); }
  emit(ev, data) { (this.listeners[ev] || []).forEach(fn => fn(data)); }

  calibrate() {
    // flight 0: excite the drone with a hand-tuned PD pilot plus dither, then identify the first model from that log
    const d = new Drone(); d.wear = this.drone.wear; const e = new Estimator(); const log = [];
    const calWind = { speed: 1, dir: 0, gust: 0.2 };
    for (let k = 0; k < 600; k++) {
      const pm = d.measure(); const x = e.update(pm); const t = k * DT;
      const ref = [6 * Math.sin(0.15 * t), 6 * Math.cos(0.11 * t), 5 + 2 * Math.sin(0.2 * t)];
      const u = calibrationPilot(x, ref, t);
      log.push({ p: pm, u }); d.step(u, calWind);
    }
    const fit = fitSindy(log);
    this.window = [log];
    this.installModel(fit.Xi, { reason: 'calibration', rmse: fit.rmse });
  }
  installModel(Xi, info) {
    this.mpc.setModel(Xi); this.modelVersion++;
    this.emit('model', { Xi, version: this.modelVersion, ...info });
  }
  // Returns 'set', or 'queued' while the drone is returning to base / swapping its battery.
  setTarget(ref) {
    const r = [Math.max(-ARENA.half, Math.min(ARENA.half, ref[0])), Math.max(-ARENA.half, Math.min(ARENA.half, ref[1])), Math.max(1, Math.min(ARENA.zMax, ref[2]))];
    if (this.mode !== 'mission') { this.pendingRef = r; return 'queued'; }
    this.ref = r; this.settled = 0; this.flightDone = false;
    return 'set';
  }
  setMode(mode, info = {}) { this.mode = mode; this.emit('mode', { mode, battery: this.drone.battery, ...info }); }
  tick() {
    if (this.mode === 'charging') return this.tickOnPad();
    const pm = this.drone.measure();
    const xPrev = this.est.x ? [...this.est.x] : null;
    const x = [...this.est.update(pm)];
    this.mpc.observe(x, xPrev, this.mpc.uPrev);
    const { u, traj } = this.mpc.solve(x, this.ref);
    const ua = this.dither ? u.map((v, c) => Math.min(U_MAX[c], Math.max(U_MIN[c], v + (c === 0 ? 2 : 1.5) * gauss()))) : u;
    this.flightLog.push({ p: pm, u: ua });
    this.drone.step(ua, this.wind);
    this.t += DT;
    const err = Math.hypot(...this.ref.map((r, i) => r - x[i])), spd = Math.hypot(x[3], x[4], x[5]);
    if (this.mode === 'mission') {
      // flight bookkeeping: a flight ends once the drone has settled on its target
      if (!this.flightDone) {
        this.settled = err < 0.6 && spd < 0.5 ? (this.settled || 0) + 1 : 0;
        if (this.settled > 15 && this.flightLog.length > 60) { this.flightDone = true; this.endFlight(); }
        else if (this.flightLog.length > 900) this.endFlight();
      }
      // low battery → return to base: climb/hold at a safe altitude over the pad first
      if (this.drone.battery < this.rtbLevel) {
        this.missionRef = this.flightDone ? null : [...this.ref];
        this.ref = [this.home[0], this.home[1], Math.max(4, Math.min(10, x[2]))];
        this.setMode('rtb');
      }
    } else if (this.mode === 'rtb') {
      if (Math.hypot(x[0] - this.home[0], x[1] - this.home[1]) < 0.8 && spd < 0.8) this.setMode('landing');
    } else if (this.mode === 'landing') {
      // descend along a gentle glide slope, aiming slightly below the pad so it actually touches down
      this.ref = [this.home[0], this.home[1], Math.max(-0.3, this.ref[2] - 0.9 * DT)];
      if (this.drone.p[2] <= 0.02 && this.ref[2] < 0.3) {
        this.endFlight();                      // landing ends a flight → SINDy refit on its log
        this.chargeT = 0; this.setMode('charging');
      }
    }
    return { x, u: ua, traj, err, wind: this.drone.wind(this.wind), mode: this.mode, battery: this.drone.battery };
  }
  tickOnPad() {
    // motors off on the pad while the battery is swapped
    this.chargeT += DT; this.t += DT;
    const x = [...this.est.update(this.drone.measure(0.002))];
    this.mpc.uPrev = [0, 0, 0];
    if (this.chargeT >= this.swapTime) {
      this.drone.battery = 1;
      this.mpc.U = null; this.mpc.d.fill(0); this.est = new Estimator();
      this.mode = 'mission';
      const next = this.pendingRef ?? this.missionRef;
      this.pendingRef = null; this.missionRef = null;
      this.ref = next ?? [this.home[0], this.home[1], 5];
      this.settled = 0; this.flightDone = !next;   // with nothing queued, hover over the pad and let the UI pick
      this.setMode('mission', { swapped: true, resumed: !!next });
    }
    return { x, u: [0, 0, 0], traj: [], err: Math.hypot(...this.ref.map((r, i) => r - x[i])), wind: this.drone.wind(this.wind), mode: this.mode, battery: this.drone.battery };
  }
  endFlight() {
    const log = this.flightLog; this.flightLog = [];
    if (log.length < 40) return;
    const before = modelErrorOnLog(this.mpc.Xi, log);
    let after = before, accepted = false;
    if (this.adaptive) {
      // fit on the rolling window + the first 70% of this flight, validate on the held-out last 30%
      const cut = Math.floor(log.length * 0.7), train = log.slice(0, cut), test = log.slice(cut - 10);
      const fit = fitSindy([...this.window, train], { prior: this.mpc.Xi });
      if (fit) {
        const oldVal = modelErrorOnLog(this.mpc.Xi, test), newVal = modelErrorOnLog(fit.Xi, test);
        // gate: only swap the model in if it predicts unseen data at least as well as the current one
        if (newVal <= oldVal * 1.02) {
          accepted = true;
          this.installModel(fit.Xi, { reason: 'refit', rmse: fit.rmse });
          after = modelErrorOnLog(fit.Xi, log);
        }
      }
      this.window.push(log); if (this.window.length > 3) this.window.shift();
    }
    this.drone.age(this.driftRate);
    const summary = { flight: this.drone.wear.flights, before, after, accepted, adaptive: this.adaptive, wear: { ...this.drone.wear } };
    this.history.push(summary); if (this.history.length > 40) this.history.shift();
    this.emit('flight', summary);
  }
  resetDrone() {
    this.drone = new Drone(); this.est = new Estimator(); this.mpc = new MPC();
    this.mode = 'mission'; this.pendingRef = null; this.missionRef = null;
    this.mpc.offsetFree = this.offsetFree ?? true;
    this.flightLog = []; this.history = []; this.window = []; this.modelVersion = 0;
    this.calibrate();
    this.emit('reset', {});
  }
}
