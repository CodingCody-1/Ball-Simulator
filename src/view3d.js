/* =============================================================================
 * View3D  —  Three.js rendering of the ball-balancing robot.
 * Coordinate mapping:  sim is Z-up (X,Y on the floor).  three.js is Y-up, so
 *   sim X -> three X,   sim Y -> three Z,   sim up -> three Y.
 * ===========================================================================*/
(function (global) {
  'use strict';
  var THREE = global.THREE;

  function View(container) {
    this.container = container;
    var w = container.clientWidth, h = container.clientHeight;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0e1116);

    this.camera = new THREE.PerspectiveCamera(50, w / h, 0.01, 100);
    this.camera.position.set(0.55, 0.45, 0.55);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(global.devicePixelRatio || 1);
    this.renderer.setSize(w, h);
    this.renderer.shadowMap.enabled = true;
    container.appendChild(this.renderer.domElement);

    this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.12, 0);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;

    // lighting
    var amb = new THREE.AmbientLight(0xffffff, 0.55);
    this.scene.add(amb);
    var dir = new THREE.DirectionalLight(0xffffff, 0.9);
    dir.position.set(0.6, 1.2, 0.4);
    dir.castShadow = true;
    dir.shadow.camera.near = 0.1; dir.shadow.camera.far = 5;
    dir.shadow.camera.left = -1; dir.shadow.camera.right = 1;
    dir.shadow.camera.top = 1; dir.shadow.camera.bottom = -1;
    this.scene.add(dir);

    // floor
    var grid = new THREE.GridHelper(4, 80, 0x3a4250, 0x222831);
    grid.position.y = 0;
    this.scene.add(grid);
    var floorMat = new THREE.ShadowMaterial({ opacity: 0.25 });
    var floor = new THREE.Mesh(new THREE.PlaneGeometry(4, 4), floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    this.scene.add(floor);

    // axes at origin for reference
    var axes = new THREE.AxesHelper(0.12);
    this.scene.add(axes);

    // ---- robot groups -----------------------------------------------------
    this.root = new THREE.Group();        // follows ball ground position
    this.scene.add(this.root);

    this.ball = new THREE.Mesh(
      new THREE.SphereGeometry(0.0508, 32, 24),
      new THREE.MeshStandardMaterial({ color: 0x2e7dd1, roughness: 0.35, metalness: 0.1 })
    );
    this.ball.castShadow = true;
    this.root.add(this.ball);

    // longitude markers so the ball's rolling is visible
    var seam = new THREE.Mesh(
      new THREE.TorusGeometry(0.0508, 0.0015, 8, 48),
      new THREE.MeshStandardMaterial({ color: 0xbfe0ff })
    );
    this.ball.add(seam);
    var seam2 = seam.clone(); seam2.rotation.y = Math.PI / 2; this.ball.add(seam2);

    // drive cage: yaws with the robot, always hugging the ball
    this.cage = new THREE.Group();
    this.root.add(this.cage);
    this.wheels = [];
    this.axleLines = [];
    var axleMat = new THREE.LineBasicMaterial({ color: 0x6b7686 });
    for (var i = 0; i < 3; i++) {
      var wheel = new THREE.Mesh(
        new THREE.CylinderGeometry(0.029, 0.029, 0.016, 20),
        new THREE.MeshStandardMaterial({ color: 0xe0a23a, roughness: 0.5 })
      );
      wheel.castShadow = true;
      this.cage.add(wheel);
      this.wheels.push(wheel);
      // axle line from the wheel center to the shared apex (shows the pyramid)
      var lg = new THREE.BufferGeometry();
      lg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
      var line = new THREE.Line(lg, axleMat);
      this.cage.add(line);
      this.axleLines.push(line);
    }
    // shared apex of the three axles (the pyramid tip)
    this.apexMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.006, 12, 8),
      new THREE.MeshStandardMaterial({ color: 0x6b7686, emissive: 0x20242c })
    );
    this.cage.add(this.apexMarker);

    // chassis: the tall body that leans. Pivots about the ball center.
    this.chassis = new THREE.Group();
    this.root.add(this.chassis);
    this.bodyMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.06, 0.075, 0.18, 24),
      new THREE.MeshStandardMaterial({ color: 0xcdd3da, roughness: 0.6, metalness: 0.2 })
    );
    this.bodyMesh.castShadow = true;
    this.chassis.add(this.bodyMesh);
    // a small mast so lean direction is legible
    var mast = new THREE.Mesh(
      new THREE.CylinderGeometry(0.006, 0.006, 0.10, 12),
      new THREE.MeshStandardMaterial({ color: 0x9aa3ad })
    );
    this.mast = mast; this.chassis.add(mast);

    // center-of-mass marker
    this.comMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.012, 16, 12),
      new THREE.MeshStandardMaterial({ color: 0xff4d6d, emissive: 0x551122 })
    );
    this.chassis.add(this.comMarker);

    // target marker on the floor
    this.target = new THREE.Mesh(
      new THREE.RingGeometry(0.02, 0.03, 24),
      new THREE.MeshBasicMaterial({ color: 0x4be08a, side: THREE.DoubleSide })
    );
    this.target.rotation.x = -Math.PI / 2;
    this.target.position.y = 0.001;
    this.scene.add(this.target);

    // external force arrow
    this.forceArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 0.0001, 0xff6633, 0.03, 0.02);
    this.scene.add(this.forceArrow);

    this._ballQuat = new THREE.Quaternion();
    this.setParams({ r: 0.0508, lBody: 0.18, rWheel: 0.029, zeta: 55 * Math.PI / 180,
                     gamma: 22 * Math.PI / 180, azimuth0: 0, comX: 0, comY: 0 });
  }

  // Rebuild sizes/positions when geometry params change.
  View.prototype.setParams = function (p) {
    this.p = Object.assign(this.p || {}, p);
    var r = this.p.r, l = this.p.lBody;

    this.ball.scale.setScalar(r / 0.0508);
    this.ball.position.y = 0;          // ball center handled by root.y below
    this.root.position.y = 0;          // root sits on floor; children use ball center at y=r

    // ball center is at y = r; put ball mesh there
    this.ball.position.set(0, r, 0);
    this.cage.position.set(0, r, 0);
    this.chassis.position.set(0, r, 0);

    // body sits l above ball center
    this.bodyMesh.position.set(0, l, 0);
    this.mast.position.set(0, l + 0.14, 0);

    // Wheel placement/orientation comes straight from the kinematics model so
    // the picture and the math never disagree. The three spin axles form a
    // pyramid sharing the apex; small pyramid angle => nearly flat wheels.
    // (sim is Z-up; map sim (x,y,z) -> three (x, z, y).)
    var kin = new BallBot.OmniKinematics(this.p);
    function toThree(v) { return new THREE.Vector3(v[0], v[2], v[1]); }
    var apex3 = toThree(kin.apex);
    for (var i = 0; i < 3; i++) {
      var w = this.wheels[i];
      var c3 = toThree(kin.center[i]);
      var ax3 = toThree(kin.axle[i]).normalize();
      w.position.copy(c3);
      w.scale.setScalar(this.p.rWheel / 0.029);
      w.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), ax3);
      // axle line from wheel center to the shared apex
      var pos = this.axleLines[i].geometry.attributes.position;
      pos.setXYZ(0, c3.x, c3.y, c3.z);
      pos.setXYZ(1, apex3.x, apex3.y, apex3.z);
      pos.needsUpdate = true;
    }
    this.apexMarker.position.copy(apex3);
    this.comMarker.position.set(this.p.comX, l, this.p.comY);
  };

  // Update transforms from sim state each frame.
  View.prototype.update = function (s, extra) {
    extra = extra || {};
    var r = this.p.r;
    // ground position
    this.root.position.x = s.pos.x;
    this.root.position.z = s.pos.y;

    // yaw applies to cage + chassis
    this.cage.rotation.set(0, 0, 0);
    this.cage.rotateY(-s.yaw);
    // chassis: yaw then lean (tip top toward +X => rot -Z; toward +Y => rot +X)
    this.chassis.quaternion.identity();
    this.chassis.rotateY(-s.yaw);
    this.chassis.rotateOnAxis(new THREE.Vector3(0, 0, 1), -s.lean.x);
    this.chassis.rotateOnAxis(new THREE.Vector3(1, 0, 0), s.lean.y);

    // ball rolling: integrate angular velocity into quaternion
    if (extra.dt) {
      // Rolling without slip: v_center = omega x (0,r,0)  =>  omega_x = v_y/r,
      // omega_z = -v_x/r. Yaw sign matches the cage (rotateY(-yaw)).
      var ox = s.vel.y / r, oy = -s.yawRate, oz = -s.vel.x / r; // three-space omega
      var ang = Math.hypot(ox, oy, oz) * extra.dt;
      if (ang > 1e-9) {
        var ax = new THREE.Vector3(ox, oy, oz).normalize();
        var dq = new THREE.Quaternion().setFromAxisAngle(ax, ang);
        this._ballQuat.premultiply(dq);
        this.ball.quaternion.copy(this._ballQuat);
      }
    }

    // target marker
    if (extra.target) {
      this.target.position.x = extra.target.x;
      this.target.position.z = extra.target.y;
    }

    // force arrow (world-space, anchored at body COM height)
    var f = extra.force || { x: 0, y: 0 };
    var mag = Math.hypot(f.x, f.y);
    if (mag > 1e-4) {
      this.forceArrow.visible = true;
      this.forceArrow.position.set(s.pos.x, r + this.p.lBody, s.pos.y);
      this.forceArrow.setDirection(new THREE.Vector3(f.x, 0, f.y).normalize());
      this.forceArrow.setLength(0.06 + 0.04 * Math.min(mag, 10), 0.03, 0.02);
    } else {
      this.forceArrow.visible = false;
    }

    // fallen tint
    this.bodyMesh.material.color.set(s.fallen ? 0xd1452e : 0xcdd3da);

    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };

  View.prototype.resize = function () {
    var w = this.container.clientWidth, h = this.container.clientHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  };

  // raycast a screen point to the floor plane -> sim (x,y) or null
  View.prototype.pickFloor = function (clientX, clientY) {
    var rect = this.renderer.domElement.getBoundingClientRect();
    var ndc = new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1);
    var ray = new THREE.Raycaster();
    ray.setFromCamera(ndc, this.camera);
    var plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    var pt = new THREE.Vector3();
    if (ray.ray.intersectPlane(plane, pt)) return { x: pt.x, y: pt.z };
    return null;
  };

  global.BallBot = global.BallBot || {};
  global.BallBot.View = View;
})(typeof window !== 'undefined' ? window : globalThis);
