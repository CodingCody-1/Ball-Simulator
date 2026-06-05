/* =============================================================================
 * ui.js  —  tiny dependency-free control-panel builder (sliders/buttons/readouts)
 * ===========================================================================*/
(function (global) {
  'use strict';

  function el(tag, cls, parent) {
    var e = document.createElement(tag);
    if (cls) e.className = cls;
    if (parent) parent.appendChild(e);
    return e;
  }

  var UI = {
    el: el,

    section: function (parent, title, collapsed) {
      var sec = el('div', 'section', parent);
      var head = el('div', 'section-head', sec);
      head.textContent = title;
      var body = el('div', 'section-body', sec);
      if (collapsed) { sec.classList.add('collapsed'); }
      head.addEventListener('click', function () { sec.classList.toggle('collapsed'); });
      return body;
    },

    // bound slider with synced number box. opts: {label,min,max,step,value,unit,fmt,onInput}
    slider: function (parent, opts) {
      var row = el('div', 'ctl', parent);
      var lab = el('label', null, row);
      lab.textContent = opts.label;
      var val = el('span', 'val', lab);
      var wrap = el('div', 'sl-wrap', row);
      var input = el('input', null, wrap);
      input.type = 'range';
      input.min = opts.min; input.max = opts.max; input.step = opts.step;
      input.value = opts.value;
      var fmt = opts.fmt || function (v) { return (+v).toFixed(3); };
      function show(v) { val.textContent = fmt(v) + (opts.unit ? ' ' + opts.unit : ''); }
      show(opts.value);
      input.addEventListener('input', function () {
        show(input.value);
        if (opts.onInput) opts.onInput(parseFloat(input.value));
      });
      return {
        set: function (v) { input.value = v; show(v); },
        get: function () { return parseFloat(input.value); },
        el: input
      };
    },

    button: function (parent, label, onClick, cls) {
      var b = el('button', 'btn ' + (cls || ''), parent);
      b.textContent = label;
      b.addEventListener('click', onClick);
      return b;
    },

    readout: function (parent, label) {
      var row = el('div', 'readout', parent);
      var l = el('span', 'rl', row); l.textContent = label;
      var v = el('span', 'rv', row);
      return { set: function (t) { v.textContent = t; }, el: v };
    },

    row: function (parent, cls) { return el('div', 'row ' + (cls || ''), parent); }
  };

  global.BallBot = global.BallBot || {};
  global.BallBot.UI = UI;
})(typeof window !== 'undefined' ? window : globalThis);
