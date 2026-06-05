/* =============================================================================
 * BallBotPlant  —  SIMULATION DYNAMICS (the "plant", NOT shipped to firmware)
 * -----------------------------------------------------------------------------
 * Models the robot as an inverted pendulum riding on a rolling ball, evaluated
 * independently on the X-Z and Y-Z planes, plus a decoupled yaw axis.
 *
 * Per-plane generalized coordinates:  phi (ball roll angle), theta (body lean)
 *   ball floor position  x = r * phi
 *
 * Lagrangian equations of motion (derived for ball + body pendulum), with a
 * motor torque `tau` applied between ball and body, a possible lateral
 * center-of-mass offset `c`, and an external horizontal force `F` on the body:
 *
 *   A*phi'' + B cos(th)*th''  - B sin(th)*th'^2  =  tau + F*r
 *   B cos(th)*phi'' + C*th''  - m*g*(l sin th + c cos th) = -tau + F*l cos(th)
 *
 *   A = (m_ball + m_body) r^2 + I_ball      (effective rolling inertia)
 *   B = m_body * r * l
 *   C = m_body * l^2 + I_body
 *
 * Integrated with classic RK4. Units are SI throughout.
 * ===========================================================================*/
(function (global) {
  'use strict';

  function BallBotPlant(p) {
    this.params = {};
    this.setParams(p || {});
    this.reset();
  }

  BallBotPlant.prototype.setParams = function (p) {
    var P = this.params;
    function def(k, d){ P[k] = (p[k] != null) ? p[k] : (P[k] != null ? P[k] : d); }
    def('r',       0.0508);  // ball radius [m]  (4" diameter)
    def('mBall',   0.6);     // ball mass [kg]
    def('iBallK',  0.6667);  // ball inertia coefficient (I = k*m*r^2; 2/3 = thin shell)
    def('mBody',   2.5);     // body mass [kg]
    def('lBody',   0.18);    // COM height above ball center [m]
    def('iBody',   0.03);    // body inertia about its COM [kg m^2]
    def('comX',    0.0);     // lateral COM offset in body frame, X [m]
    def('comY',    0.0);     // lateral COM offset in body frame, Y [m]
    def('g',       9.81);    // gravity [m/s^2]
    def('iYaw',    0.02);    // yaw inertia [kg m^2]
    def('yawDamp', 0.02);    // yaw viscous damping
    def('floorDamp', 0.0);   // rolling resistance on ball velocity
    def('maxLean', 1.2);     // [rad] hard fall-over angle; sim freezes the body past this
  };

  BallBotPlant.prototype.reset = function (state) {
    // per-plane: [phi, phiDot, theta, thetaDot]
    this.X = [0, 0, (state && state.leanX) || 0, 0];
    this.Y = [0, 0, (state && state.leanY) || 0, 0];
    this.yaw = 0; this.yawRate = 0;
    this.fallen = false;
    // external force [N] applied to body COM, world frame, and a timer
    this.extForce = { x: 0, y: 0 };
  };

  // Inertia helpers (recomputed each step so live param edits take effect).
  BallBotPlant.prototype._coeffs = function (c) {
    var P = this.params;
    var iBall = P.iBallK * P.mBall * P.r * P.r;
    var A = (P.mBall + P.mBody) * P.r * P.r + iBall;
    var B = P.mBody * P.r * P.lBody;
    var C = P.mBody * P.lBody * P.lBody + P.iBody;
    return { A: A, B: B, C: C, m: P.mBody, g: P.g, r: P.r, l: P.lBody, c: c };
  };

  // Plane derivative: state=[phi,phiDot,theta,thetaDot], tau motor torque,
  // F external horizontal force, c lateral COM offset for this plane.
  BallBotPlant.prototype._deriv = function (st, tau, F, c) {
    var k = this._coeffs(c);
    var th = st[2], thd = st[3];
    var s = Math.sin(th), co = Math.cos(th);
    var Bc = k.B * co;
    var det = k.A * k.C - Bc * Bc;
    if (Math.abs(det) < 1e-12) det = 1e-12;
    var rhs1 = tau + k.B * s * thd * thd + F * k.r;
    var rhs2 = -tau + k.m * k.g * (k.l * s + c * co) + F * k.l * co;
    var phidd = (k.C * rhs1 - Bc * rhs2) / det;
    var thdd  = (k.A * rhs2 - Bc * rhs1) / det;
    return [st[1], phidd, st[3], thdd];
  };

  BallBotPlant.prototype._rk4 = function (st, tau, F, c, dt) {
    var d = this._deriv, self = this;
    function add(a, b, h){ return [a[0]+b[0]*h, a[1]+b[1]*h, a[2]+b[2]*h, a[3]+b[3]*h]; }
    var k1 = d.call(self, st, tau, F, c);
    var k2 = d.call(self, add(st, k1, dt/2), tau, F, c);
    var k3 = d.call(self, add(st, k2, dt/2), tau, F, c);
    var k4 = d.call(self, add(st, k3, dt), tau, F, c);
    return [
      st[0] + dt/6*(k1[0]+2*k2[0]+2*k3[0]+k4[0]),
      st[1] + dt/6*(k1[1]+2*k2[1]+2*k3[1]+k4[1]),
      st[2] + dt/6*(k1[2]+2*k2[2]+2*k3[2]+k4[2]),
      st[3] + dt/6*(k1[3]+2*k2[3]+2*k3[3]+k4[3])
    ];
  };

  /**
   * Advance the plant by dt seconds under the given plane torques.
   * @param {{x:number,y:number,yaw:number}} tau  motor torques from controller
   * @param {number} dt
   */
  BallBotPlant.prototype.step = function (tau, dt) {
    var P = this.params;
    // Apply rolling resistance as a small opposing torque on the ball.
    var dampX = -P.floorDamp * this.X[1];
    var dampY = -P.floorDamp * this.Y[1];

    this.X = this._rk4(this.X, tau.x + dampX, this.extForce.x, P.comX, dt);
    this.Y = this._rk4(this.Y, tau.y + dampY, this.extForce.y, P.comY, dt);

    // Yaw: simple rigid-body with damping.
    var yawAcc = (tau.yaw - P.yawDamp * this.yawRate) / Math.max(P.iYaw, 1e-6);
    this.yawRate += yawAcc * dt;
    this.yaw     += this.yawRate * dt;

    // Fall detection: clamp once tipped past maxLean so the sim doesn't explode.
    var leanMag = Math.hypot(this.X[2], this.Y[2]);
    if (leanMag > P.maxLean) {
      this.fallen = true;
      this.X[2] = clampSign(this.X[2], P.maxLean);
      this.Y[2] = clampSign(this.Y[2], P.maxLean);
      this.X[3] *= 0.2; this.Y[3] *= 0.2;
    }
  };

  function clampSign(v, m){ return v > m ? m : (v < -m ? -m : v); }

  /** Snapshot the measurable state for the controller / view. */
  BallBotPlant.prototype.getState = function () {
    var r = this.params.r;
    return {
      pos:      { x: r * this.X[0], y: r * this.Y[0] },
      vel:      { x: r * this.X[1], y: r * this.Y[1] },
      lean:     { x: this.X[2], y: this.Y[2] },
      leanRate: { x: this.X[3], y: this.Y[3] },
      phi:      { x: this.X[0], y: this.Y[0] },  // raw ball roll (for rendering)
      yaw:      this.yaw,
      yawRate:  this.yawRate,
      fallen:   this.fallen
    };
  };

  /** Apply an instantaneous external impulse (kick) to the body COM. */
  BallBotPlant.prototype.applyImpulse = function (jx, jy) {
    // Impulse -> change in body COM velocity -> change in theta-dot & phi-dot.
    // Approximate: distribute into lean rate via pendulum length.
    var P = this.params;
    var dThd = jx / Math.max(P.mBody * P.lBody, 1e-6);
    this.X[3] += dThd;
    this.X[1] += jx / Math.max((P.mBall + P.mBody) * P.r, 1e-6);
    var dThdY = jy / Math.max(P.mBody * P.lBody, 1e-6);
    this.Y[3] += dThdY;
    this.Y[1] += jy / Math.max((P.mBall + P.mBody) * P.r, 1e-6);
  };

  var api = { Plant: BallBotPlant };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  global.BallBot = global.BallBot || {};
  global.BallBot.Plant = BallBotPlant;
})(typeof window !== 'undefined' ? window : globalThis);
