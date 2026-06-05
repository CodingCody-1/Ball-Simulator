/* =============================================================================
 * autotune.js  —  automatic controller-gain tuner
 * -----------------------------------------------------------------------------
 * Runs the SAME plant + controller headlessly through a battery of scenarios,
 * scores each set of gains with a cost function, and searches for the gains
 * that minimise it. Because it optimises against whatever physical parameters
 * you pass in, you can re-tune after changing the ball, body, COM, gravity, or
 * torque limit.
 *
 * Algorithm: Hooke-Jeeves style coordinate (pattern) search on a handful of
 * gains, seeded from the current (known-stable) gains and using multiplicative
 * steps so values stay positive. Derivative-free and dependency-free.
 *
 * Works in Node (require) and the browser (window.BallBot). The optimiser is
 * resumable one "pass" at a time so the browser can run it without freezing.
 * ===========================================================================*/
(function (global) {
  'use strict';
  var BB = global.BallBot || (typeof require !== 'undefined' ? {
    Plant: require('./physics.js').Plant,
    Controller: require('./controller.js').Controller,
    DEFAULT_GAINS: require('./controller.js').DEFAULT_GAINS
  } : {});

  var DT = 1 / 200;

  // ---- scenarios: each returns the metrics we care about -------------------
  // w = cost weights for {settle, over(shoot), lean, err, effort}.
  var SCENARIOS = [
    { name: 'recover12',  T: 3.5, lean0: 12 * Math.PI / 180,
      w: { settle: 1.0, lean: 2.0, err: 30, over: 0, effort: 0.03 } },
    { name: 'step0.4',    T: 5.0, target: function () { return { x: 0.4, y: 0, yaw: 0 }; },
      w: { settle: 1.0, lean: 1.0, err: 30, over: 25, effort: 0.03 } },
    { name: 'kick1.5',    T: 5.0, kick: { t: 0.4, x: 1.5 },
      w: { settle: 0.8, lean: 3.0, err: 20, over: 0, effort: 0.02 } },
    { name: 'kick3.0',    T: 5.0, kick: { t: 0.4, x: 3.0 },
      w: { settle: 0.5, lean: 2.5, err: 12, over: 0, effort: 0.02 } },
    { name: 'comOffset',  T: 5.0, params: { comX: 0.01 },
      w: { settle: 0.2, lean: 0, err: 120, over: 0, effort: 0 } }
  ];
  // Soft barrier: discourage any lean excursion past this (margin before tipping).
  var LEAN_SOFT = 0.45;     // ~26 deg
  var LEAN_BARRIER = 350;   // cost per rad beyond the soft limit

  function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

  // Run a single scenario, return metrics. Detects blow-ups / falls.
  function runScenario(sc, params, gains) {
    var p = Object.assign({}, params, sc.params || {});
    var plant = new BB.Plant(p);
    plant.reset({ leanX: sc.lean0 || 0 });
    var ctrl = new BB.Controller(gains);
    var steps = Math.round(sc.T / DT);
    var peakLean = 0, peakTau = 0, settle = 0, overshoot = 0, fell = false;
    var tgtEnd = sc.target ? sc.target(sc.T) : { x: 0, y: 0 };
    for (var i = 0; i < steps; i++) {
      var t = i * DT;
      var s = plant.getState();
      s.target = sc.target ? sc.target(t) : { x: 0, y: 0, yaw: 0 };
      if (sc.kick && Math.abs(t - sc.kick.t) < DT / 2) plant.applyImpulse(sc.kick.x || 0, 0);
      var out = ctrl.update(s, DT);
      plant.step(out.tau, DT);
      var lean = Math.hypot(s.lean.x, s.lean.y);
      if (!isFinite(lean) || lean > 1.0) { fell = true; break; }
      peakLean = Math.max(peakLean, lean);
      peakTau = Math.max(peakTau, Math.abs(out.tau.x), Math.abs(out.tau.y));
      var e = Math.hypot(s.pos.x - tgtEnd.x, s.pos.y - tgtEnd.y);
      if (e > 0.02) settle = t;                    // last time outside 2 cm band
      if (sc.target && tgtEnd.x > 0.01) overshoot = Math.max(overshoot, s.pos.x - tgtEnd.x);
    }
    if (plant.fallen) fell = true;
    var f = plant.getState();
    var finalErr = Math.hypot(f.pos.x - tgtEnd.x, f.pos.y - tgtEnd.y);
    if (!isFinite(finalErr)) { fell = true; finalErr = 1; }
    return { settle: fell ? sc.T : settle + DT, peakLean: peakLean, peakTau: peakTau,
             overshoot: overshoot, finalErr: finalErr, fell: fell };
  }

  // Total cost across scenarios for a candidate gain set.
  function evalCost(tuned, params, baseGains) {
    var gains = Object.assign({}, baseGains, tuned);
    var total = 0, breakdown = {};
    for (var k = 0; k < SCENARIOS.length; k++) {
      var sc = SCENARIOS[k], m = runScenario(sc, params, gains), w = sc.w;
      var c = (m.fell ? 1000 : 0)
            + w.settle * m.settle
            + (w.over || 0) * m.overshoot
            + (w.lean || 0) * m.peakLean
            + (w.err || 0) * m.finalErr
            + (w.effort || 0) * m.peakTau
            + (m.peakLean > LEAN_SOFT ? (m.peakLean - LEAN_SOFT) * LEAN_BARRIER : 0);
      breakdown[sc.name] = c;
      total += c;
    }
    return { cost: total, breakdown: breakdown };
  }

  // ---- optimiser (resumable pattern search) -------------------------------
  var TUNE_KEYS = ['kAngle', 'kAngleRate', 'kPos', 'vMax', 'kVel', 'kPosInt'];
  var BOUNDS = {
    kAngle: [1, 20], kAngleRate: [0.05, 2.0], kPos: [0.3, 5.0],
    vMax: [0.2, 2.0], kVel: [0.05, 0.8], kPosInt: [0.0, 0.4]
  };

  function Optimizer(params, baseGains) {
    this.params = params;
    this.base = Object.assign({}, BB.DEFAULT_GAINS, baseGains);
    this.x = {};
    for (var i = 0; i < TUNE_KEYS.length; i++) {
      var k = TUNE_KEYS[i];
      this.x[k] = this.base[k] != null ? this.base[k] : BB.DEFAULT_GAINS[k];
    }
    this.factor = 1.6;
    this.minFactor = 1.04;
    this.evals = 0;
    this.startCost = this._cost(this.x);
    this.bestCost = this.startCost;
    this.done = false;
  }
  Optimizer.prototype._cost = function (x) {
    this.evals++;
    return evalCost(x, this.params, this.base).cost;
  };
  // One full coordinate sweep (~12 evaluations). Returns progress snapshot.
  Optimizer.prototype.pass = function () {
    var improved = false;
    for (var i = 0; i < TUNE_KEYS.length; i++) {
      var k = TUNE_KEYS[i], b = BOUNDS[k];
      var dirs = [this.factor, 1 / this.factor];
      for (var d = 0; d < dirs.length; d++) {
        if (k === 'kPosInt' && this.x[k] === 0) continue;
        var trial = Object.assign({}, this.x);
        trial[k] = clamp(this.x[k] * dirs[d], b[0], b[1]);
        if (trial[k] === this.x[k]) continue;
        var c = this._cost(trial);
        if (c < this.bestCost - 1e-9) { this.x = trial; this.bestCost = c; improved = true; break; }
      }
    }
    if (!improved) this.factor = Math.sqrt(this.factor);  // refine the step
    if (this.factor < this.minFactor) this.done = true;
    return { gains: Object.assign({}, this.x), cost: this.bestCost,
             startCost: this.startCost, factor: this.factor,
             evals: this.evals, done: this.done };
  };
  // Run to completion synchronously (Node / tests).
  Optimizer.prototype.run = function (maxPasses) {
    var snap;
    for (var i = 0; i < (maxPasses || 200) && !this.done; i++) snap = this.pass();
    return snap || { gains: Object.assign({}, this.x), cost: this.bestCost,
                     startCost: this.startCost, evals: this.evals, done: true };
  };

  // Browser-friendly async driver: yields between passes so the UI stays live.
  function autoTune(params, baseGains, opts) {
    opts = opts || {};
    var opt = new Optimizer(params, baseGains);
    return new Promise(function (resolve) {
      function loop() {
        var snap = opt.pass();
        if (opts.onProgress) opts.onProgress(snap);
        if (snap.done) return resolve(snap);
        setTimeout(loop, 0);
      }
      setTimeout(loop, 0);
    });
  }

  var api = { Optimizer: Optimizer, autoTune: autoTune, evalCost: evalCost,
              runScenario: runScenario, SCENARIOS: SCENARIOS, TUNE_KEYS: TUNE_KEYS };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  global.BallBot = global.BallBot || {};
  global.BallBot.AutoTune = api;
})(typeof window !== 'undefined' ? window : globalThis);
