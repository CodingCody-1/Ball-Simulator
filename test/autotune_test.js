/* Verify the auto-tuner: it should lower the cost from the hand-tuned defaults,
 * stay stable, and adapt when the physical configuration changes.
 * Run: node test/autotune_test.js */
const { Optimizer, evalCost } = require('../src/autotune.js');
const { DEFAULT_GAINS } = require('../src/controller.js');

function tune(label, params, baseOverride) {
  const t0 = Date.now();
  const base = Object.assign({}, DEFAULT_GAINS, baseOverride || {});
  const opt = new Optimizer(params, base);
  const r = opt.run();
  const ms = Date.now() - t0;
  console.log(`\n## ${label}`);
  console.log(`  cost ${r.startCost.toFixed(2)} -> ${r.cost.toFixed(2)}` +
    `  (${(100 * (1 - r.cost / r.startCost)).toFixed(0)}% better)  ` +
    `${r.evals} evals, ${ms} ms`);
  const g = r.gains;
  console.log('  gains: ' + Object.keys(g).map(k => `${k}=${g[k].toFixed(3)}`).join('  '));
  return r;
}

// Default robot
tune('default robot', {});

// Heavier, taller body (harder to balance) — gains should change
tune('heavy tall body (mBody=6, lBody=0.30)', { mBody: 6, lBody: 0.30 });

// Light, short body
tune('light short body (mBody=1, lBody=0.10)', { mBody: 1, lBody: 0.10 });

// Low gravity (moon-ish)
tune('low gravity (g=3)', { g: 3 });

// Tight torque limit (tauLimit is a controller gain, so override the base)
tune('weak motors (tauLimit=1.5)', {}, { tauLimit: 1.5 });

console.log('\nSanity: re-scoring default tuned gains on default robot ' +
  'should match the optimizer best cost.');
const opt = new Optimizer({}, DEFAULT_GAINS);
const best = opt.run();
const rescore = evalCost(best.gains, {}, DEFAULT_GAINS).cost;
console.log(`  best=${best.cost.toFixed(3)}  rescore=${rescore.toFixed(3)}  ` +
  (Math.abs(best.cost - rescore) < 1e-6 ? 'OK' : 'MISMATCH'));
