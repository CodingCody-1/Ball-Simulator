/* Headless verification of the control loop + plant + kinematics.
 * Run: node test/sim_test.js
 * Confirms the robot recovers from an initial tilt, holds position, drives to
 * a commanded point, rejects a kick, and survives a lateral COM offset. */
const { Controller } = require('../src/controller.js');
const { Plant } = require('../src/physics.js');
const { OmniKinematics } = require('../src/kinematics.js');

function run(label, opts) {
  const plant = new Plant(opts.params || {});
  plant.reset({ leanX: opts.lean0 || 0, leanY: 0 });
  const ctrl = new Controller(opts.gains || {});
  const kin = new OmniKinematics({ r: plant.params.r });

  const dt = 1 / 200;
  const T = opts.T || 6;
  const steps = Math.round(T / dt);
  let maxLean = 0, fellAt = -1;
  let maxWheel = 0;

  for (let i = 0; i < steps; i++) {
    const t = i * dt;
    const s = plant.getState();
    s.target = opts.target ? opts.target(t) : { x: 0, y: 0, yaw: 0 };
    if (opts.kick && Math.abs(t - opts.kick.t) < dt / 2) {
      plant.applyImpulse(opts.kick.x || 0, opts.kick.y || 0);
    }
    const out = ctrl.update(s, dt);
    // exercise kinematics allocation path
    const world = kin.planeTorqueToWorld(out.tau);
    const wt = kin.wheelTorques(world);
    maxWheel = Math.max(maxWheel, ...wt.map(Math.abs));
    plant.step(out.tau, dt);

    const lean = Math.hypot(s.lean.x, s.lean.y);
    maxLean = Math.max(maxLean, lean);
    if (plant.fallen && fellAt < 0) fellAt = t;
  }

  const f = plant.getState();
  // With a lateral COM offset the *correct* equilibrium lean is atan(c/l), so
  // compare against that expected lean rather than zero.
  const lExp = Math.atan2(-(plant.params.comX || 0), plant.params.lBody);
  const settled = Math.hypot(f.lean.x - lExp, f.lean.y);
  const posErr = opts.target
    ? Math.hypot(f.pos.x - opts.target(T).x, f.pos.y - opts.target(T).y)
    : Math.hypot(f.pos.x, f.pos.y);
  // A continuously moving target (circle) is allowed steady lag; everything
  // else must converge tightly to the setpoint.
  const posTol = opts.posTol != null ? opts.posTol : 0.03;
  const leanTol = opts.leanTol != null ? opts.leanTol : 0.03;
  const ok = !plant.fallen && settled < leanTol && posErr < posTol;
  console.log(
    `${ok ? 'PASS' : 'FAIL'}  ${label.padEnd(34)} ` +
    `maxLean=${(maxLean*57.3).toFixed(1)}deg  finalLean=${(settled*57.3).toFixed(2)}deg ` +
    `posErr=${(posErr*100).toFixed(2)}cm  maxWheelTau=${maxWheel.toFixed(2)}Nm` +
    (fellAt >= 0 ? `  FELL@${fellAt.toFixed(2)}s` : ''));
  return ok;
}

let all = true;
all &= run('recover from 10deg tilt', { lean0: 10 * Math.PI / 180 });
all &= run('recover from 15deg tilt', { lean0: 15 * Math.PI / 180 });
all &= run('hold origin (quiescent)', {});
all &= run('drive to (0.5, 0.3)', { T: 8, target: () => ({ x: 0.5, y: 0.3, yaw: 0 }) });
all &= run('reject 1.5 N.s kick', { T: 9, kick: { t: 1.0, x: 1.5 } });
all &= run('hold with 8mm COM offset', { T: 8, params: { comX: 0.008 } });
// Slow circle: a continuously moving target produces bounded steady lag, so we
// only require that it stays upright and tracks within a loose radius.
all &= run('slow circle trajectory', {
  T: 12, posTol: 0.12, leanTol: 0.12,
  target: (t) => ({ x: 0.25 * Math.cos(0.4 * t), y: 0.25 * Math.sin(0.4 * t), yaw: 0 })
});

console.log('\n' + (all ? 'ALL PASS' : 'SOME FAILED'));
process.exit(all ? 0 : 1);
