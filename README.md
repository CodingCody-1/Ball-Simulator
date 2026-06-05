# Ballbot Simulator

A web-based simulator for a **ball-balancing robot** (ballbot): a 4″ ball driven by
**3 omni wheels**, with an inverted-pendulum body kept upright by a feedback
control loop. Runs entirely in the browser — no build step.

![overview](docs/overview.png)

## Run it

**Option A — just open it.** Double-click `index.html`. (Three.js loads from a CDN,
so you need an internet connection the first time.)

**Option B — local server** (avoids any browser file:// quirks):

```
node server.js
# then open http://localhost:5173
```

**Option C — GitHub Pages.** This is a static site with no build step, so it hosts
directly: push to GitHub, then in **Settings → Pages** set *Source: Deploy from a
branch*, branch `main`, folder `/ (root)`. Your simulator goes live at
`https://<user>.github.io/<repo>/`. (Three.js loads over HTTPS from a CDN, which Pages
serves happily. `server.js` is only for local previewing and isn't used by Pages.)

## What you can do

- **Watch it balance.** The robot keeps itself upright by rolling the ball under its
  center of mass, exactly like a real ballbot.
- **Command positions.** Drag on the 3D floor, use the Target X/Y/yaw sliders, or the
  Home / Random buttons. The robot drives to the point and holds it.
- **Move the center of mass.** Adjust COM height and lateral X/Y offset. A lateral
  offset makes the robot lean to a new equilibrium angle (`atan(-offset / height)`)
  while still holding position — the integral term absorbs the bias.
- **Apply external forces.** Kick buttons apply impulses; the constant-force sliders
  apply a steady push. Watch it recover.
- **Adjust every parameter** live: ball size/mass/inertia, body mass/inertia, gravity,
  rolling resistance, wheel radius, wheel zenith & tilt angle, and all controller gains.
- **Auto-tune the controller** for the current robot (see below).
- **Read telemetry** and a live plot of lean angle and position error.

## Auto-tune

The **✨ Auto-tune for this robot** button (in the Controller-gains section) searches
for controller gains that work well *for whatever physical configuration is currently
set*. Change the body mass, COM height, gravity, or torque limit, click auto-tune, and
it finds gains suited to that robot — usually in under a second, with the simulation
still running.

How it works: it runs the same plant + controller headlessly through a battery of
scenarios (tilt recovery, a point-to-point move, two kicks, and a COM-offset hold),
scores each candidate with a cost function (settle time, overshoot, peak lean, steady
error, control effort, with a soft barrier against large lean excursions), and
minimises it with a derivative-free pattern search seeded from the current gains. It's
in `src/autotune.js` (also dependency-free / portable) and runnable headlessly:

```
node test/autotune_test.js     # tune several configurations and report
```

Example: the default robot tunes ~40% lower cost (faster, lower-overshoot moves); a
heavier, taller body automatically gets a stiffer balance loop and more integral
authority; weak motors get a lower cruise speed.

## Project layout

```
index.html         page + styling, loads everything
server.js          zero-dependency static server (for Option B)
src/
  controller.js    >>> PORTABLE control law (no dependencies) <<<
  kinematics.js    >>> PORTABLE omni-wheel allocation (no dependencies) <<<
  autotune.js      headless gain optimiser (portable; powers the Auto-tune button)
  physics.js       simulation-only plant (inverted-pendulum-on-ball dynamics)
  view3d.js        Three.js rendering
  ui.js            tiny control-panel builder
  main.js          wires plant + controller + kinematics + view together
test/
  sim_test.js      headless pass/fail verification (node test/sim_test.js)
  metrics.js       quantitative tuning metrics (node test/metrics.js)
  autotune_test.js auto-tuner verification across configs (node test/autotune_test.js)
```

## Porting the control loop to real hardware

This was a hard requirement, so the controller is deliberately isolated.
**`src/controller.js` and `src/kinematics.js` are the only files you port.** They:

- have **zero dependencies** (no browser, no Three.js, no DOM) — plain ECMAScript,
- run unchanged in the browser, in Node, and translate 1:1 to C/C++/Rust,
- use **SI units** throughout with documented sign conventions,
- expose a fixed-rate `update(state, dt)` entry point.

### Firmware loop sketch

```js
const { Controller } = require('./controller.js');
const { OmniKinematics } = require('./kinematics.js');

const ctrl = new Controller(/* optional gain overrides */);
const kin  = new OmniKinematics({ r: 0.0508, rWheel: 0.029,
                                  zeta: 47*Math.PI/180, tilt: 45*Math.PI/180 });

// at a fixed rate (e.g. 200 Hz):
function onTick(dt) {
  const state = {
    pos:      readBallPosition(),     // {x,y}  [m]
    vel:      readBallVelocity(),     // {x,y}  [m/s]
    lean:     readIMUTilt(),          // {x,y}  [rad]
    leanRate: readIMURate(),          // {x,y}  [rad/s]
    yaw:      readYaw(), yawRate: readYawRate(),
    target:   getSetpoint()           // {x,y,yaw}
  };
  const out = ctrl.update(state, dt);                 // -> ball torque {x,y,yaw}
  const world  = kin.planeTorqueToWorld(out.tau);     // -> world torque vector
  const wheel  = kin.wheelTorques(world);             // -> [tau0, tau1, tau2]
  commandMotors(wheel);                               // send to your ESCs
}
```

The simulator's `physics.js` (the "plant") is the *only* thing you throw away on
real hardware — it's replaced by the actual robot and your sensors.

## Control architecture

A **cascaded loop**, evaluated independently on the two tilt planes plus a decoupled
yaw axis:

```
position ─[outer: vel-profiled P + I]→ desired lean ─[inner P-D]→ ball torque ─[allocator]→ 3 wheel torques
```

- **Inner (balance) loop** — fast P-D on lean angle error (~18 rad/s, well damped).
- **Outer (position) loop** — *velocity-profiled*: position error sets a **speed
  command** (saturated at `vMax`), and lean is driven by **velocity error**. This
  decouples cruise speed from lean, so point-to-point moves accelerate, cruise
  near-upright, then decelerate — instead of slamming the lean to its limit.
- **Target-velocity feedforward** — the controller differentiates the setpoint
  (clamped, so a step command can't spike) so moving trajectories track without lag.
- **Anti-windup integral** — a small integral that only acts when near the target
  *and* nearly stopped, so it cancels steady biases (e.g. a lateral COM offset, which
  needs a steady holding lean) without winding up during travel.

The outer loop runs well below the inner-loop bandwidth because the plant is
**non-minimum-phase** — to move forward the robot must first lean backward.

Default gains are tuned and verified headlessly. `node test/sim_test.js` is a
pass/fail suite; `node test/metrics.js` reports the quantitative tuning metrics
(peak lean, settle time, overshoot, kick recovery, trajectory lag). Typical results:
a 0.5 m move peaks at ~13° lean with ~3 cm overshoot and settles in ~3 s; an 8 mm
COM offset is held to ~0.4 cm. Every gain is also adjustable live in the UI.

## Physics model

Each tilt plane is modeled as an inverted pendulum riding on a rolling ball, derived
via Lagrangian mechanics and integrated with RK4:

```
A·φ̈ + B·cosθ·θ̈ − B·sinθ·θ̇²              =  τ + F·r
B·cosθ·φ̈ + C·θ̈ − m·g·(l·sinθ + c·cosθ)   = −τ + F·l·cosθ
```

where `A = (m_ball+m_body)·r² + I_ball`, `B = m_body·r·l`, `C = m_body·l² + I_body`,
`τ` is the motor torque, `c` the lateral COM offset, and `F` an external force.
