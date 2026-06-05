/* =============================================================================
 * main.js  —  app wiring: plant + controller + kinematics + 3D view + UI loop
 * ===========================================================================*/
(function (global) {
  'use strict';
  var BB = global.BallBot;
  var UI = BB.UI;
  var DEG = Math.PI / 180, RAD2DEG = 180 / Math.PI;

  // ---- model state ---------------------------------------------------------
  var params = {
    r: 0.0508, mBall: 0.6, iBallK: 0.6667, mBody: 2.5, lBody: 0.18, iBody: 0.03,
    comX: 0, comY: 0, g: 9.81, iYaw: 0.02, yawDamp: 0.02, floorDamp: 0.0,
    rWheel: 0.029, zeta: 55 * DEG, gamma: 22 * DEG, azimuth0: 0, maxLean: 1.2
  };
  var commands = { tx: 0, ty: 0, tyaw: 0 };
  var forces = { fx: 0, fy: 0 };
  var sim = { running: true, controlHz: 200, rtf: 1.0 };

  // ---- engine objects ------------------------------------------------------
  var plant = new BB.Plant(params);
  var controller = new BB.Controller();
  var kin = new BB.OmniKinematics(params);
  var view = new BB.View(document.getElementById('view'));
  view.setParams(params);

  var lastOut = { tau: { x: 0, y: 0, yaw: 0 }, leanDes: { x: 0, y: 0 } };
  var wheelSpeeds = [0, 0, 0], wheelTorques = [0, 0, 0];
  var simClock = 0;

  // ---- one control tick (fixed dt) ----------------------------------------
  function controlTick(dt) {
    var s = plant.getState();
    s.target = { x: commands.tx, y: commands.ty, yaw: commands.tyaw };
    var out = controller.update(s, dt);
    lastOut = out;

    var omega = kin.ballOmega(s.vel.x, s.vel.y, s.yawRate);
    wheelSpeeds = kin.wheelSpeeds(omega);
    wheelTorques = kin.wheelTorques(kin.planeTorqueToWorld(out.tau));

    plant.extForce.x = forces.fx;
    plant.extForce.y = forces.fy;
    plant.step(out.tau, dt);
    simClock += dt;
    pushPlot(simClock, Math.hypot(s.lean.x, s.lean.y) * RAD2DEG,
             Math.hypot(s.pos.x - commands.tx, s.pos.y - commands.ty));
  }

  // ---- fixed-timestep loop -------------------------------------------------
  var acc = 0, prev = performance.now();
  function frame(now) {
    var real = Math.min((now - prev) / 1000, 0.05);
    prev = now;
    var dt = 1 / sim.controlHz;
    var elapsed = 0;                       // sim-time advanced this frame
    if (sim.running && !plant.fallen) {
      acc += real * sim.rtf;
      var n = 0;
      while (acc >= dt && n < 2000) { controlTick(dt); acc -= dt; n++; }
      elapsed = n * dt;
    } else {
      acc = 0;
    }
    var s = plant.getState();
    view.update(s, { dt: elapsed, target: commands, force: forces, wheelSpeeds: wheelSpeeds });
    updateTelemetry(s);
    drawPlot();
    requestAnimationFrame(frame);
  }

  // =========================================================================
  //  UI
  // =========================================================================
  var panel = document.getElementById('panel');

  // ---- simulation control --------------------------------------------------
  var simBody = UI.section(panel, '▶  Simulation');
  var rowS = UI.row(simBody);
  var playBtn = UI.button(rowS, 'Pause', function () {
    sim.running = !sim.running;
    playBtn.textContent = sim.running ? 'Pause' : 'Play';
    prev = performance.now();
  }, 'primary');
  UI.button(rowS, 'Reset', doReset);
  UI.button(rowS, 'Nudge tilt', function () { plant.X[2] += 8 * DEG; });
  UI.slider(simBody, { label: 'Real-time factor', min: 0.1, max: 2, step: 0.1, value: 1,
    fmt: function (v) { return (+v).toFixed(1) + '×'; },
    onInput: function (v) { sim.rtf = v; } });
  UI.slider(simBody, { label: 'Control rate', min: 50, max: 500, step: 10, value: 200,
    unit: 'Hz', fmt: function (v) { return (+v).toFixed(0); },
    onInput: function (v) { sim.controlHz = v; } });

  // ---- commands ------------------------------------------------------------
  var cmdBody = UI.section(panel, '⌖  Position command');
  var hint = UI.el('div', 'hint', cmdBody);
  hint.textContent = 'Drag the 3D floor (Shift+drag rotates camera) to set target.';
  var sTx = UI.slider(cmdBody, { label: 'Target X', min: -1, max: 1, step: 0.01, value: 0,
    unit: 'm', fmt: f2, onInput: function (v) { commands.tx = v; } });
  var sTy = UI.slider(cmdBody, { label: 'Target Y', min: -1, max: 1, step: 0.01, value: 0,
    unit: 'm', fmt: f2, onInput: function (v) { commands.ty = v; } });
  var sTyaw = UI.slider(cmdBody, { label: 'Target yaw', min: -180, max: 180, step: 1, value: 0,
    unit: '°', fmt: f0, onInput: function (v) { commands.tyaw = v * DEG; } });
  var rowC = UI.row(cmdBody);
  UI.button(rowC, 'Home (0,0)', function () { setTarget(0, 0); });
  UI.button(rowC, 'Random', function () {
    setTarget((Math.random() * 2 - 1) * 0.6, (Math.random() * 2 - 1) * 0.6);
  });

  // ---- external forces -----------------------------------------------------
  var fBody = UI.section(panel, '💥  External disturbance');
  var rowK = UI.row(fBody);
  UI.button(rowK, '← Kick', function () { plant.applyImpulse(-1.5, 0); });
  UI.button(rowK, 'Kick →', function () { plant.applyImpulse(1.5, 0); });
  UI.button(rowK, '↑ Kick', function () { plant.applyImpulse(0, 1.5); });
  UI.button(rowK, 'Kick ↓', function () { plant.applyImpulse(0, -1.5); });
  UI.slider(fBody, { label: 'Constant force X', min: -15, max: 15, step: 0.5, value: 0,
    unit: 'N', fmt: f1, onInput: function (v) { forces.fx = v; } });
  UI.slider(fBody, { label: 'Constant force Y', min: -15, max: 15, step: 0.5, value: 0,
    unit: 'N', fmt: f1, onInput: function (v) { forces.fy = v; } });

  // ---- center of mass ------------------------------------------------------
  var comBody = UI.section(panel, '⚖  Center of mass');
  UI.slider(comBody, { label: 'COM height (l)', min: 0.05, max: 0.4, step: 0.005, value: params.lBody,
    unit: 'm', fmt: f3, onInput: function (v) { params.lBody = v; syncParams(); } });
  UI.slider(comBody, { label: 'COM offset X', min: -0.05, max: 0.05, step: 0.001, value: 0,
    unit: 'm', fmt: f3, onInput: function (v) { params.comX = v; syncParams(); } });
  UI.slider(comBody, { label: 'COM offset Y', min: -0.05, max: 0.05, step: 0.001, value: 0,
    unit: 'm', fmt: f3, onInput: function (v) { params.comY = v; syncParams(); } });

  // ---- physical parameters -------------------------------------------------
  var phBody = UI.section(panel, '🔧  Physical parameters', true);
  UI.slider(phBody, { label: 'Ball diameter', min: 2, max: 10, step: 0.25, value: 4,
    unit: 'in', fmt: f2, onInput: function (v) { params.r = v * 0.0254 / 2; syncParams(); } });
  UI.slider(phBody, { label: 'Ball mass', min: 0.05, max: 3, step: 0.05, value: params.mBall,
    unit: 'kg', fmt: f2, onInput: function (v) { params.mBall = v; syncParams(); } });
  UI.slider(phBody, { label: 'Ball inertia k (I=k m r²)', min: 0.3, max: 1.0, step: 0.01, value: params.iBallK,
    fmt: f2, onInput: function (v) { params.iBallK = v; syncParams(); } });
  UI.slider(phBody, { label: 'Body mass', min: 0.2, max: 10, step: 0.1, value: params.mBody,
    unit: 'kg', fmt: f2, onInput: function (v) { params.mBody = v; syncParams(); } });
  UI.slider(phBody, { label: 'Body inertia', min: 0.005, max: 0.3, step: 0.005, value: params.iBody,
    unit: 'kg·m²', fmt: f3, onInput: function (v) { params.iBody = v; syncParams(); } });
  UI.slider(phBody, { label: 'Gravity', min: 0, max: 20, step: 0.1, value: params.g,
    unit: 'm/s²', fmt: f2, onInput: function (v) { params.g = v; syncParams(); } });
  UI.slider(phBody, { label: 'Rolling resistance', min: 0, max: 0.02, step: 0.0005, value: 0,
    fmt: f3, onInput: function (v) { params.floorDamp = v; syncParams(); } });

  // ---- wheel geometry ------------------------------------------------------
  var wBody = UI.section(panel, '⚙  Omni-wheel geometry', true);
  UI.slider(wBody, { label: 'Wheel radius', min: 0.01, max: 0.06, step: 0.001, value: params.rWheel,
    unit: 'm', fmt: f3, onInput: function (v) { params.rWheel = v; syncParams(); } });
  UI.slider(wBody, { label: 'Contact zenith (yaw↔drive balance)', min: 20, max: 80, step: 1, value: 55,
    unit: '°', fmt: f0, onInput: function (v) { params.zeta = v * DEG; syncParams(); } });
  UI.slider(wBody, { label: 'Pyramid angle (lower = flatter wheels)', min: 8, max: 80, step: 1, value: 22,
    unit: '°', fmt: f0, onInput: function (v) { params.gamma = v * DEG; syncParams(); } });
  UI.slider(wBody, { label: 'Array rotation', min: 0, max: 120, step: 1, value: 0,
    unit: '°', fmt: f0, onInput: function (v) { params.azimuth0 = v * DEG; syncParams(); } });

  // ---- controller gains ----------------------------------------------------
  var gBody = UI.section(panel, '🎛  Controller gains', true);
  var g = controller.gains;
  var gainRefs = {};
  gainSlider('Balance P (kAngle)', 'kAngle', 0, 15, 0.1);
  gainSlider('Balance D (kAngleRate)', 'kAngleRate', 0, 2, 0.01);
  gainSlider('Position→speed (kPos)', 'kPos', 0, 5, 0.05);
  gainSlider('Max speed (vMax)', 'vMax', 0.1, 2.5, 0.05);
  gainSlider('Speed→lean (kVel)', 'kVel', 0, 1.0, 0.01);
  gainSlider('Position I (kPosInt)', 'kPosInt', 0, 0.5, 0.01);
  gainSlider('Lean limit', 'leanLimit', 0.05, 0.6, 0.01);
  gainSlider('Yaw P (kYaw)', 'kYaw', 0, 3, 0.05);
  gainSlider('Yaw D (kYawRate)', 'kYawRate', 0, 1, 0.01);
  gainSlider('Torque limit', 'tauLimit', 0.5, 12, 0.1);
  var gRow = UI.row(gBody);
  var tuneBtn = UI.button(gRow, '✨ Auto-tune for this robot', runAutoTune, 'primary');
  UI.button(gRow, 'Reset to defaults', function () {
    controller.setGains(BB.DEFAULT_GAINS); controller.reset(); rebuildGainSliders();
    tuneStatus.set('gains reset to defaults');
  });
  var tuneStatus = UI.readout(gBody, 'Auto-tune');
  tuneStatus.set('searches gains for the current parameters');

  function runAutoTune() {
    if (!BB.AutoTune) { tuneStatus.set('autotune module not loaded'); return; }
    tuneBtn.disabled = true;
    var startGains = Object.assign({}, controller.gains);
    tuneStatus.set('tuning… (optimizing against current robot)');
    BB.AutoTune.autoTune(params, startGains, {
      onProgress: function (snap) {
        tuneStatus.set('tuning… cost ' + snap.startCost.toFixed(1) + ' → ' +
          snap.cost.toFixed(1) + '  (' + snap.evals + ' evals)');
      }
    }).then(function (res) {
      controller.setGains(res.gains);
      controller.reset();
      rebuildGainSliders();
      var pct = Math.max(0, Math.round(100 * (1 - res.cost / res.startCost)));
      tuneStatus.set('done — ' + pct + '% better (cost ' +
        res.startCost.toFixed(1) + ' → ' + res.cost.toFixed(1) + '), applied');
      tuneStatus.el.style.color = '#7ee787';
      tuneBtn.disabled = false;
    }).catch(function (e) {
      tuneStatus.set('error: ' + e.message); tuneBtn.disabled = false;
    });
  }
  function gainSlider(label, key, min, max, step) {
    var ref = UI.slider(gBody, { label: label, min: min, max: max, step: step, value: g[key],
      fmt: f3, onInput: function (v) { var o = {}; o[key] = v; controller.setGains(o); } });
    gainRefs[key] = ref;
  }
  function rebuildGainSliders() {
    for (var k in gainRefs) gainRefs[k].set(controller.gains[k]);
  }

  // =========================================================================
  //  Telemetry + plot
  // =========================================================================
  var telBody = UI.section(panel, '📈  Telemetry');
  var tLean = UI.readout(telBody, 'Lean (X, Y)');
  var tPos = UI.readout(telBody, 'Position');
  var tVel = UI.readout(telBody, 'Speed');
  var tLeanDes = UI.readout(telBody, 'Commanded lean');
  var tTau = UI.readout(telBody, 'Ball torque (X,Y,yaw)');
  var tWspd = UI.readout(telBody, 'Wheel speeds');
  var tWtau = UI.readout(telBody, 'Wheel torques');
  var tStatus = UI.readout(telBody, 'Status');

  function updateTelemetry(s) {
    tLean.set(f1(s.lean.x * RAD2DEG) + ', ' + f1(s.lean.y * RAD2DEG) + ' °');
    tPos.set(f1(s.pos.x * 100) + ', ' + f1(s.pos.y * 100) + ' cm');
    tVel.set(f2(Math.hypot(s.vel.x, s.vel.y)) + ' m/s');
    tLeanDes.set(f1(lastOut.leanDes.x * RAD2DEG) + ', ' + f1(lastOut.leanDes.y * RAD2DEG) + ' °');
    tTau.set(f2(lastOut.tau.x) + ', ' + f2(lastOut.tau.y) + ', ' + f2(lastOut.tau.yaw) + ' N·m');
    tWspd.set(wheelSpeeds.map(function (w) { return f1(w); }).join(', ') + ' rad/s');
    tWtau.set(wheelTorques.map(function (w) { return f2(w); }).join(', ') + ' N·m');
    tStatus.set(s.fallen ? 'FALLEN — press Reset' : (sim.running ? 'balancing' : 'paused'));
    tStatus.el.style.color = s.fallen ? '#ff6b6b' : '#7ee787';
  }

  // simple scrolling plot
  var plotCanvas = document.getElementById('plot');
  var pctx = plotCanvas.getContext('2d');
  var PLOT_T = 8; // seconds window
  var leanHist = [], posHist = [];
  function pushPlot(t, leanDeg, posErrM) {
    leanHist.push({ t: t, v: leanDeg });
    posHist.push({ t: t, v: posErrM * 100 });
    while (leanHist.length && leanHist[0].t < t - PLOT_T) leanHist.shift();
    while (posHist.length && posHist[0].t < t - PLOT_T) posHist.shift();
  }
  function drawPlot() {
    var w = plotCanvas.width, h = plotCanvas.height;
    pctx.clearRect(0, 0, w, h);
    pctx.fillStyle = '#0a0d12'; pctx.fillRect(0, 0, w, h);
    // gridlines
    pctx.strokeStyle = '#1c2230'; pctx.lineWidth = 1;
    pctx.beginPath(); pctx.moveTo(0, h / 2); pctx.lineTo(w, h / 2); pctx.stroke();
    var t1 = simClock, t0 = t1 - PLOT_T;
    function drawTrace(hist, scale, color) {
      pctx.strokeStyle = color; pctx.lineWidth = 1.5; pctx.beginPath();
      for (var i = 0; i < hist.length; i++) {
        var x = (hist[i].t - t0) / PLOT_T * w;
        var y = h / 2 - hist[i].v * scale;
        if (y < 1) y = 1; if (y > h - 1) y = h - 1;
        if (i === 0) pctx.moveTo(x, y); else pctx.lineTo(x, y);
      }
      pctx.stroke();
    }
    drawTrace(leanHist, h / 2 / 25, '#ffb347');   // lean: +/-25 deg full scale
    drawTrace(posHist, h / 2 / 50, '#4be08a');    // pos err: +/-50 cm full scale
    pctx.fillStyle = '#ffb347'; pctx.font = '11px monospace';
    pctx.fillText('lean (±25°)', 6, 14);
    pctx.fillStyle = '#4be08a';
    pctx.fillText('pos err (±50cm)', 6, 28);
  }

  // =========================================================================
  //  interactions
  // =========================================================================
  function setTarget(x, y) {
    commands.tx = x; commands.ty = y; sTx.set(x); sTy.set(y);
  }
  function syncParams() {
    plant.setParams(params);
    kin.set(params);
    view.setParams(params);
  }
  function doReset() {
    plant.setParams(params); plant.reset();
    controller.reset();
    simClock = 0; leanHist = []; posHist = [];
    view._ballQuat.identity(); view.ball.quaternion.identity();
    prev = performance.now(); acc = 0;
  }

  // drag on floor to set target (without Shift, which orbits the camera)
  var dragging = false;
  var dom = view.renderer.domElement;
  dom.addEventListener('pointerdown', function (e) {
    if (e.shiftKey) return;       // shift => let OrbitControls rotate
    var p = view.pickFloor(e.clientX, e.clientY);
    if (p) { dragging = true; view.controls.enabled = false; setTarget(clampM(p.x), clampM(p.y)); }
  });
  dom.addEventListener('pointermove', function (e) {
    if (!dragging) return;
    var p = view.pickFloor(e.clientX, e.clientY);
    if (p) setTarget(clampM(p.x), clampM(p.y));
  });
  window.addEventListener('pointerup', function () { dragging = false; view.controls.enabled = true; });
  function clampM(v) { return Math.max(-1, Math.min(1, v)); }

  window.addEventListener('resize', function () {
    view.resize();
    plotCanvas.width = plotCanvas.clientWidth; plotCanvas.height = plotCanvas.clientHeight;
  });
  plotCanvas.width = plotCanvas.clientWidth; plotCanvas.height = plotCanvas.clientHeight;

  // ---- number formatters ---------------------------------------------------
  function f0(v) { return (+v).toFixed(0); }
  function f1(v) { return (+v).toFixed(1); }
  function f2(v) { return (+v).toFixed(2); }
  function f3(v) { return (+v).toFixed(3); }

  // debug handle
  global.__bb = { plant: plant, controller: controller, kin: kin, view: view,
    params: params, commands: commands, get clock(){ return simClock; } };

  // go
  doReset();
  requestAnimationFrame(frame);
})(typeof window !== 'undefined' ? window : globalThis);
