/* =============================================================================
 * OmniKinematics  —  3-OMNI-WHEEL <-> BALL TORQUE / VELOCITY ALLOCATION
 * -----------------------------------------------------------------------------
 * Portable (browser + Node + transpilable). No external dependencies.
 *
 * GEOMETRY (parameterized so every value is adjustable from the UI):
 *   Three omni wheels press on a sphere of radius r. Wheel i sits at:
 *      azimuth      psi_i = azimuth0 + i * 120 deg     (around the vertical axis)
 *      zenith       zeta                                (angle from +Z, the top)
 *   Each wheel's drive direction is tilted by `tilt` (alpha) out of the local
 *   "downhill" tangent, which is what gives the array yaw authority.
 *
 *   For wheel i we build the unit sensitivity vector n_i so that:
 *      omega_wheel_i = (r / r_wheel) * ( n_i . omega_ball )           (velocity)
 *      tau_ball      = sum_i (r / r_wheel) * n_i * tau_wheel_i = J^T * tau_wheel
 *   where the rows of J are (r / r_wheel) * n_i.
 *
 *   n_i = -sin(a)cos(z) * e_r  +  cos(a) * e_t  +  sin(a)sin(z) * e_z
 *      e_r = (cos psi, sin psi, 0)   radial-horizontal
 *      e_t = (-sin psi, cos psi, 0)  tangential-horizontal
 *      e_z = (0,0,1)
 *
 * This file exposes BOTH the forward map (ball motion -> wheel speeds, for
 * telemetry) and the inverse torque map (desired ball torque -> wheel torques,
 * the real motor commands you would send to firmware).
 * ===========================================================================*/
(function (global) {
  'use strict';

  var DEG = Math.PI / 180;

  function OmniKinematics(p) {
    this.set(p || {});
  }

  OmniKinematics.prototype.set = function (p) {
    this.r       = p.r       != null ? p.r       : 0.0508; // ball radius [m]
    this.rWheel  = p.rWheel  != null ? p.rWheel  : 0.029;  // omni wheel radius [m]
    this.zeta    = p.zeta    != null ? p.zeta    : 75*DEG; // zenith angle [rad] (lower = flatter wheels)
    this.tilt    = p.tilt    != null ? p.tilt    : 72*DEG; // wheel tilt (yaw authority) [rad]
    this.azimuth0= p.azimuth0!= null ? p.azimuth0: 0;      // first wheel azimuth [rad]
    this._build();
  };

  OmniKinematics.prototype._build = function () {
    var z = this.zeta, a = this.tilt, k = this.r / this.rWheel;
    this.n = [];   // sensitivity unit vectors
    this.J = [];   // 3x3 Jacobian rows = k * n_i
    this.contact = []; // contact point unit vectors (for 3D drawing)
    for (var i = 0; i < 3; i++) {
      var psi = this.azimuth0 + i * 120 * DEG;
      var cr = Math.cos(psi), sr = Math.sin(psi);
      var er = [cr, sr, 0];
      var et = [-sr, cr, 0];
      var ez = [0, 0, 1];
      var ni = [
        -Math.sin(a)*Math.cos(z)*er[0] + Math.cos(a)*et[0] + Math.sin(a)*Math.sin(z)*ez[0],
        -Math.sin(a)*Math.cos(z)*er[1] + Math.cos(a)*et[1] + Math.sin(a)*Math.sin(z)*ez[1],
        -Math.sin(a)*Math.cos(z)*er[2] + Math.cos(a)*et[2] + Math.sin(a)*Math.sin(z)*ez[2]
      ];
      this.n.push(ni);
      this.J.push([k*ni[0], k*ni[1], k*ni[2]]);
      // contact point on the sphere surface (zenith z, azimuth psi)
      this.contact.push([
        Math.sin(z)*cr, Math.sin(z)*sr, Math.cos(z)
      ]);
    }
    this._Jt_inv = invert3(transpose3(this.J)); // (J^T)^{-1} for torque allocation
  };

  /** Forward: ball angular velocity [rad/s] -> 3 wheel speeds [rad/s]. */
  OmniKinematics.prototype.wheelSpeeds = function (omegaBall) {
    return mul3(this.J, omegaBall);
  };

  /** Ball angular velocity from floor velocity & yaw rate (rolling, no slip).
   *  v_center = r (omega_y, -omega_x, 0)  ->  omega = (-vy/r, vx/r, yawRate). */
  OmniKinematics.prototype.ballOmega = function (vx, vy, yawRate) {
    return [-vy / this.r, vx / this.r, yawRate];
  };

  /** Inverse: desired ball torque vector (world) -> 3 wheel torques [N*m]. */
  OmniKinematics.prototype.wheelTorques = function (tauBallVec) {
    return mul3(this._Jt_inv, tauBallVec);
  };

  /** Map controller plane torques {x,y,yaw} to a world ball-torque vector.
   *  tau.x accelerates +X motion  -> torque about +Y axis.
   *  tau.y accelerates +Y motion  -> torque about -X axis.
   *  tau.yaw                       -> torque about +Z axis. */
  OmniKinematics.prototype.planeTorqueToWorld = function (tau) {
    return [-tau.y, tau.x, tau.yaw];
  };

  // ---- tiny 3x3 linear algebra (no deps) ----------------------------------
  function transpose3(M){return [[M[0][0],M[1][0],M[2][0]],[M[0][1],M[1][1],M[2][1]],[M[0][2],M[1][2],M[2][2]]];}
  function mul3(M, v){return [
    M[0][0]*v[0]+M[0][1]*v[1]+M[0][2]*v[2],
    M[1][0]*v[0]+M[1][1]*v[1]+M[1][2]*v[2],
    M[2][0]*v[0]+M[2][1]*v[1]+M[2][2]*v[2]];}
  function invert3(m){
    var a=m[0][0],b=m[0][1],c=m[0][2],d=m[1][0],e=m[1][1],f=m[1][2],g=m[2][0],h=m[2][1],i=m[2][2];
    var A=e*i-f*h, B=-(d*i-f*g), C=d*h-e*g;
    var det=a*A+b*B+c*C;
    if (Math.abs(det)<1e-12) det = 1e-12;
    var id=1/det;
    return [
      [A*id, (c*h-b*i)*id, (b*f-c*e)*id],
      [B*id, (a*i-c*g)*id, (c*d-a*f)*id],
      [C*id, (b*g-a*h)*id, (a*e-b*d)*id]
    ];
  }

  var api = { OmniKinematics: OmniKinematics };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  global.BallBot = global.BallBot || {};
  global.BallBot.OmniKinematics = OmniKinematics;
})(typeof window !== 'undefined' ? window : globalThis);
