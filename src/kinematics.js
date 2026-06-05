/* =============================================================================
 * OmniKinematics  —  3-OMNI-WHEEL <-> BALL TORQUE / VELOCITY ALLOCATION
 * -----------------------------------------------------------------------------
 * Portable (browser + Node + transpilable). No external dependencies.
 *
 * GEOMETRY (real ballbot arrangement, parameterized for the UI):
 *   Three omni wheels press on a sphere of radius r. Wheel i:
 *      azimuth psi_i = azimuth0 + i*120 deg          (around the vertical axis)
 *      contact zenith `zeta`                          (angle from +Z, the top)
 *      spin axle tilted `gamma` from vertical, IN the wheel's vertical plane.
 *   Because every axle lies in a vertical plane, the three axles all intersect
 *   at one point on the central axis -> they form a pyramid sharing an apex.
 *   The wheel disc is perpendicular to its axle, so a small `gamma` makes the
 *   wheels lie nearly parallel to the floor (as on real ballbots).
 *
 *   The tangent rolling/drive direction then works out to the pure azimuthal
 *   tangent e_t, giving the sensitivity (depends only on the contact zenith):
 *      n_i = (r / r_wheel) * ( -cos(zeta) * e_r_i  +  sin(zeta) * e_z )
 *      omega_wheel_i = n_i . omega_ball                              (velocity)
 *      tau_ball      = sum_i n_i * tau_wheel_i = J^T * tau_wheel      (torque)
 *   where the rows of J are n_i.  The shared sin(zeta)*e_z term means spinning
 *   all three wheels together rotates the ball about the vertical axis (yaw);
 *   the differential -cos(zeta)*e_r term produces translation.
 *      e_r = (cos psi, sin psi, 0)   radial-horizontal
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
    this.zeta    = p.zeta    != null ? p.zeta    : 55*DEG; // CONTACT zenith on ball [rad]
    this.gamma   = (p.gamma != null ? p.gamma : (p.tilt != null ? p.tilt : 22*DEG)); // axle tilt from vertical (pyramid half-angle) [rad]
    this.azimuth0= p.azimuth0!= null ? p.azimuth0: 0;      // first wheel azimuth [rad]
    this._build();
  };

  // Real ballbot geometry: each wheel's spin axle lies in its own vertical
  // plane, tilted from vertical by gamma, so the three axles meet at a common
  // apex on the central axis (a pyramid). The wheel disc is perpendicular to
  // the axle, so small gamma => wheels nearly parallel to the floor.
  //
  // Drive direction works out to the azimuthal tangent e_t, giving sensitivity
  //   n_i = (R/r_w) * ( -cos(zeta)*e_r_i + sin(zeta)*e_z )
  // (depends only on the CONTACT zenith, not on the pyramid angle). The shared
  // sin(zeta)*e_z term is what makes common-mode wheel spin = yaw.
  OmniKinematics.prototype._build = function () {
    var z = this.zeta, g = this.gamma, k = this.r / this.rWheel, R = this.r, rw = this.rWheel;
    var sz = Math.sin(z), cz = Math.cos(z), sg = Math.sin(g), cg = Math.cos(g);
    this.n = [];        // sensitivity vectors (already include R/r_w)
    this.J = [];        // 3x3 Jacobian rows = n_i
    this.contact = [];  // contact point UNIT vectors on the ball (for drawing)
    this.axle = [];     // wheel spin-axle UNIT vectors (for drawing)
    this.center = [];   // wheel center positions [m] (for drawing)
    for (var i = 0; i < 3; i++) {
      var psi = this.azimuth0 + i * 120 * DEG;
      var cr = Math.cos(psi), sr = Math.sin(psi);
      var er = [cr, sr, 0];           // radial-horizontal
      // sensitivity row
      var ni = [k * (-cz * er[0]), k * (-cz * er[1]), k * sz];
      this.n.push(ni);
      this.J.push([ni[0], ni[1], ni[2]]);
      // contact point (unit) at zenith z, azimuth psi
      this.contact.push([sz * cr, sz * sr, cz]);
      // Spin axle (unit): tilted gamma from vertical but leaning INWARD toward
      // the central axis, so the three axles converge at a shared apex ABOVE
      // the ball (a pyramid). This also puts the wheel center just OUTSIDE the
      // sphere with the rim sitting tangent on the ball at the contact point.
      this.axle.push([-sg * cr, -sg * sr, cg]);
      // wheel center = R*contact + rWheel*(cos g * e_r + sin g * e_z)
      var wr = R * sz + rw * cg;                    // radial distance of center
      this.center.push([wr * cr, wr * sr, R * cz + rw * sg]);
    }
    // apex (shared point of all three axles) on the central axis, above the ball
    var H = sg > 1e-6 ? (R * cz + (rw + R * sz * cg) / sg) : 1e6;
    this.apex = [0, 0, H];
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
