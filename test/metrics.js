/* Quantitative tuning harness. Reports the metrics that define "good feel":
 * peak lean, settle time, overshoot, kick recovery, trajectory RMS lag.
 * Run: node test/metrics.js  (optionally G='{"kVel":0.3,...}' to override gains) */
const { Controller } = require('../src/controller.js');
const { Plant } = require('../src/physics.js');

const dt = 1 / 200;
const gains = process.env.G ? JSON.parse(process.env.G) : {};

function sim(opts) {
  const plant = new Plant(opts.params || {});
  plant.reset({ leanX: opts.lean0 || 0 });
  const ctrl = new Controller(gains);
  const T = opts.T || 8, steps = Math.round(T / dt);
  let peakLean = 0, peakSpeed = 0, lagSq = 0, lagN = 0;
  const log = [];
  for (let i = 0; i < steps; i++) {
    const t = i * dt;
    const s = plant.getState();
    s.target = opts.target ? opts.target(t) : { x: 0, y: 0, yaw: 0 };
    if (opts.kick && Math.abs(t - opts.kick.t) < dt / 2) plant.applyImpulse(opts.kick.x || 0, 0);
    const out = ctrl.update(s, dt);
    plant.step(out.tau, dt);
    const lean = Math.hypot(s.lean.x, s.lean.y);
    peakLean = Math.max(peakLean, lean);
    peakSpeed = Math.max(peakSpeed, Math.hypot(s.vel.x, s.vel.y));
    if (opts.target) {
      const tg = opts.target(t);
      lagSq += (s.pos.x - tg.x) ** 2 + (s.pos.y - tg.y) ** 2; lagN++;
    }
    log.push({ t, px: s.pos.x, py: s.pos.y });
  }
  // settle time: last time |pos-target| exceeds tol
  const tg = opts.target ? opts.target(T) : { x: 0, y: 0 };
  const tol = opts.tol || 0.02;
  let settle = 0, overshoot = 0;
  for (const r of log) {
    const e = Math.hypot(r.px - tg.x, r.py - tg.y);
    if (e > tol) settle = r.t;
    // overshoot for 1-D moves along x
    if (opts.target && tg.x > 0.01) overshoot = Math.max(overshoot, r.px - tg.x);
  }
  return {
    peakLeanDeg: peakLean * 57.3,
    settle: settle + dt,
    overshootCm: overshoot * 100,
    peakSpeed,
    rmsLagCm: lagN ? Math.sqrt(lagSq / lagN) * 100 : 0,
    finalErrCm: Math.hypot(log[log.length - 1].px - tg.x, log[log.length - 1].py - tg.y) * 100,
    fallen: plant.fallen
  };
}

function show(label, m) {
  console.log(`${label.padEnd(22)} peakLean=${m.peakLeanDeg.toFixed(1).padStart(5)}°  ` +
    `settle=${m.settle.toFixed(2)}s  overshoot=${m.overshootCm.toFixed(1)}cm  ` +
    `vmax=${m.peakSpeed.toFixed(2)}m/s  finalErr=${m.finalErrCm.toFixed(2)}cm` +
    (m.rmsLagCm ? `  rmsLag=${m.rmsLagCm.toFixed(1)}cm` : '') +
    (m.fallen ? '  FELL' : ''));
}

show('step 0.5m', sim({ target: () => ({ x: 0.5, y: 0, yaw: 0 }) }));
// step commanded mid-run (like a UI button press) -- exercises the FF clamp
show('step 0.5m @t=1s', sim({ T: 7, tol: 0.02,
  target: (t) => ({ x: t < 1 ? 0 : 0.5, y: 0, yaw: 0 }) }));
show('step 1.0m', sim({ T: 10, target: () => ({ x: 1.0, y: 0, yaw: 0 }) }));
show('recover 10deg', sim({ lean0: 10 * Math.PI / 180 }));
show('recover 20deg', sim({ lean0: 20 * Math.PI / 180 }));
show('kick 1.5Ns', sim({ T: 8, kick: { t: 0.5, x: 1.5 } }));
show('kick 3.0Ns', sim({ T: 8, kick: { t: 0.5, x: 3.0 } }));
show('COM offset 1cm', sim({ T: 8, params: { comX: 0.01 } }));
show('circle r0.3 0.6rad/s', sim({ T: 12,
  target: (t) => ({ x: 0.3 * Math.cos(0.6 * t), y: 0.3 * Math.sin(0.6 * t), yaw: 0 }) }));
