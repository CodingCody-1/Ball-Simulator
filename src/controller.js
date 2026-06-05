/* =============================================================================
 * BallBotController  —  PORTABLE BALANCE + POSITION CONTROL LAW
 * -----------------------------------------------------------------------------
 * This file is intentionally free of ANY dependency on the browser, Three.js,
 * the DOM, or the simulator. It is plain ECMAScript that runs unchanged in:
 *    - the browser (loaded as a classic <script> -> window.BallBot.Controller)
 *    - Node.js          (require('./controller.js').Controller)
 *    - a transpiler to C/C++/Rust for embedded firmware (1:1 mapping of math)
 *
 * PORTING NOTES
 *   The controller is a cascaded loop, evaluated independently on two
 *   orthogonal tilt planes (X and Y) plus a decoupled yaw axis:
 *
 *       outer (position) loop:  posErr -> desired lean angle  (theta_des)
 *       inner (balance)  loop:  (theta - theta_des) -> ball torque
 *
 *   To port: call `update(input, dt)` at a fixed rate (e.g. 200 Hz) with the
 *   measured state, read `out.tau.{x,y,yaw}` and feed it to your allocator
 *   (see kinematics.js -> ballTorqueToWheelTorques). Everything is SI units.
 *
 *   Sign conventions (right-handed world, +Z up):
 *     pos.x / pos.y  : ball contact-point position on the floor [m]
 *     lean.x         : body tilt in the X-Z plane, +lean tips the top toward +X [rad]
 *     lean.y         : body tilt in the Y-Z plane, +lean tips the top toward +Y [rad]
 *     tau.x          : ball drive torque that accelerates motion along +X [N*m]
 *     tau.y          : ball drive torque that accelerates motion along +Y [N*m]
 *     tau.yaw        : ball drive torque about the vertical axis [N*m]
 * ===========================================================================*/
(function (global) {
  'use strict';

  function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

  // ---- Default, hand-tuned gains (verified stable in test/sim_test.js) -------
  var DEFAULT_GAINS = {
    // Inner balance loop (acts on lean angle error). Strong + fast.
    kAngle:    7.0,   // [N*m / rad]      P on (lean - lean_des)
    kAngleRate:0.60,  // [N*m / (rad/s)]  D on lean rate

    // Outer loop: a velocity-profiled position controller. Position error sets
    // a *speed* command (saturated at vMax); lean is driven by *velocity*
    // error. This decouples cruise speed from lean, so point-to-point moves
    // accelerate, cruise near-upright, then decelerate -- instead of slamming
    // the lean to its limit. Runs well below the inner-loop bandwidth because
    // the plant is non-minimum-phase (it must lean "the wrong way" to start).
    kPos:      1.3,   // [1/s]            position error -> velocity command
    vMax:      0.65,  // [m/s]            cruise-speed cap
    kVel:      0.24,  // [rad / (m/s)]    velocity error -> commanded lean
    kPosInt:   0.12,  // [rad / (m*s)]    I on position error (kills COM-offset bias)
    intBand:   0.30,  // [m]              only integrate within this of target (anti-windup)
    intVelGate:0.10,  // [m/s]            ...and only when nearly stopped (no move windup)
    ffTau:     0.05,  // [s]              low-pass time const for target-velocity FF
    leanLimit: 0.30,  // [rad]            clamp on commanded lean (~17 deg)

    // Yaw axis (fully decoupled, simple PD).
    kYaw:      0.8,   // [N*m / rad]
    kYawRate:  0.12,  // [N*m / (rad/s)]

    // Actuator limit applied to the final ball torque per axis.
    tauLimit:  4.0,   // [N*m]
    intLimit:  0.25   // [rad] anti-windup clamp on the integrated lean command
  };

  function BallBotController(gains) {
    this.gains = Object.assign({}, DEFAULT_GAINS, gains || {});
    this.reset();
  }

  BallBotController.prototype.reset = function () {
    this._intX = 0;
    this._intY = 0;
    this._lastTgtX = null; this._lastTgtY = null;  // for target-velocity feedforward
    this._tgtVelX = 0; this._tgtVelY = 0;
    // Exposed for telemetry / plotting.
    this.leanDes = { x: 0, y: 0 };
  };

  BallBotController.prototype.setGains = function (gains) {
    Object.assign(this.gains, gains);
  };

  /**
   * One control tick.
   * @param {Object} s  measured state:
   *    { pos:{x,y}, vel:{x,y}, lean:{x,y}, leanRate:{x,y}, yaw, yawRate,
   *      target:{x,y,yaw} }
   * @param {number} dt timestep [s]
   * @returns { tau:{x,y,yaw}, leanDes:{x,y} }
   */
  BallBotController.prototype.update = function (s, dt) {
    var g = this.gains;

    // ---- Outer loop: velocity-profiled position control, per axis ----------
    function outer(pos, vel, target, tgtVel, self, axis) {
      var posErr = pos - target;                         // >0 => past the target
      // position error -> speed command toward the target, saturated at vMax,
      // plus feedforward of the target's own velocity so moving setpoints
      // (trajectories) are tracked without lag.
      var vDes = tgtVel - clamp(g.kPos * posErr, -g.vMax, g.vMax);
      var velErr = vel - vDes;

      // Integrate position error ONLY near the target. During long moves the
      // error is large for a while and would wind up, causing overshoot; the
      // integral exists only to cancel steady biases (e.g. a lateral COM
      // offset, which needs a small steady holding lean) once we have arrived.
      var integ = self['_int' + axis];
      if (Math.abs(posErr) < g.intBand && Math.abs(vel) < g.intVelGate) {
        integ += posErr * dt;
        integ = clamp(integ, -g.intLimit / Math.max(g.kPosInt, 1e-9),
                              g.intLimit / Math.max(g.kPosInt, 1e-9));
      }
      self['_int' + axis] = integ;

      // Lean to drive the velocity error to zero (lean toward the way we travel).
      var leanDes = -(g.kVel * velErr + g.kPosInt * integ);
      return clamp(leanDes, -g.leanLimit, g.leanLimit);
    }

    // Estimate target velocity (low-pass filtered finite difference) for FF.
    // The raw difference is clamped to vMax so that a *step* change of the
    // setpoint (a button press, a drag, a slider) does not inject a huge
    // one-tick velocity spike -- only genuine trajectory motion feeds forward.
    var aFF = dt / (g.ffTau + dt);
    if (this._lastTgtX !== null) {
      var rawX = clamp((s.target.x - this._lastTgtX) / dt, -g.vMax, g.vMax);
      var rawY = clamp((s.target.y - this._lastTgtY) / dt, -g.vMax, g.vMax);
      this._tgtVelX += aFF * (rawX - this._tgtVelX);
      this._tgtVelY += aFF * (rawY - this._tgtVelY);
    }
    this._lastTgtX = s.target.x; this._lastTgtY = s.target.y;

    var leanDesX = outer(s.pos.x, s.vel.x, s.target.x, this._tgtVelX, this, 'X');
    var leanDesY = outer(s.pos.y, s.vel.y, s.target.y, this._tgtVelY, this, 'Y');
    this.leanDes.x = leanDesX;
    this.leanDes.y = leanDesY;

    // ---- Inner loop: balance about the commanded lean -----------------------
    var tauX = g.kAngle * (s.lean.x - leanDesX) + g.kAngleRate * s.leanRate.x;
    var tauY = g.kAngle * (s.lean.y - leanDesY) + g.kAngleRate * s.leanRate.y;

    // ---- Yaw axis -----------------------------------------------------------
    var yawErr = s.yaw - (s.target.yaw || 0);
    var tauYaw = -(g.kYaw * yawErr + g.kYawRate * s.yawRate);

    return {
      tau: {
        x:   clamp(tauX,   -g.tauLimit, g.tauLimit),
        y:   clamp(tauY,   -g.tauLimit, g.tauLimit),
        yaw: clamp(tauYaw, -g.tauLimit, g.tauLimit)
      },
      leanDes: { x: leanDesX, y: leanDesY }
    };
  };

  // ---- export -------------------------------------------------------------
  var api = { Controller: BallBotController, DEFAULT_GAINS: DEFAULT_GAINS };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  global.BallBot = global.BallBot || {};
  global.BallBot.Controller = BallBotController;
  global.BallBot.DEFAULT_GAINS = DEFAULT_GAINS;
})(typeof window !== 'undefined' ? window : globalThis);
