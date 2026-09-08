// Run the real replay logic with a minimal DOM; canvas drawing is checked visually.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');

function element() {
  return {
    value: '', textContent: '', disabled: false, handlers: {},
    style: { setProperty() {} }, classList: { toggle() {} },
    setAttribute() {}, replaceChildren() {}, getContext: () => ({ clearRect() {} }),
    addEventListener(name, callback) { this.handlers[name] = callback; },
  };
}
const html = fs.readFileSync(`${__dirname}/index.html`, 'utf8');
const nodes = Object.fromEntries([...html.matchAll(/id="([^"]+)"/g)].map(m => [m[1], element()]));
nodes.speed.value = '1';
const pending = [];
const context = vm.createContext({
  document: {
    getElementById: id => { assert.ok(nodes[id], `Unknown element: ${id}`); return nodes[id]; },
    querySelectorAll: () => [], createElement: element, createTextNode: element,
    addEventListener() {},
  },
  ResizeObserver: class { observe() {} }, requestAnimationFrame() {},
  console: { error() {} },
  fetch: url => new Promise(resolve => pending.push({ url, resolve })),
});
const run = code => vm.runInContext(code, context);
const state = (height, contact = 0) => [0, height, 0, 0, 0, 0, contact, contact];
const dataset = {
  changeStep: 1, thrustScale: .7,
  episodes: [0, 7].map(seed => ({ seed, group: 'Development', start: [1], passed: true, margin: 1,
    final: state(0, 1), frames: [
      { s: state(1), a: 2, gain: 18, nextGain: 14, ms: 1, p: [], status: 'solved', iterations: 50, fractions: [.3, .1, .6, 0], slack: .01, violation: .0001 },
      { s: state(.5), a: 0, gain: 14, nextGain: 13, ms: 1, p: [], status: 'contact', iterations: 0, fractions: [1, 0, 0, 0], slack: 0, violation: 0 },
    ],
  })),
};
async function reply(request, ok = true) {
  assert.ok(request, 'Expected a flight request');
  request.resolve({ ok, status: ok ? 200 : 404, json: async () => dataset });
  await new Promise(setImmediate);
}

(async () => {
  run(fs.readFileSync(`${__dirname}/app.js`, 'utf8'));
  await reply(pending.shift());
  run("scenario = 'nominal'; load()"); await reply(pending.shift());
  assert.match(nodes['insight-copy'].textContent, /Small model changes/);
  run("controller = 'fixed'; load()"); await reply(pending.shift());
  assert.match(nodes['insight-copy'].textContent, /stays fixed/, 'Controller switch must refresh text even when the title is unchanged');

  run('selectFlight(7)');
  run("scenario = 'fault'; load()"); const older = pending.shift();
  run("controller = 'adaptive'; load()"); const newer = pending.shift();
  await reply(newer); await reply(older);
  assert.equal(run('flight.seed'), 7, 'Rapid switches must retain the selected seed');
  assert.match(nodes['load-status'].textContent, /Adaptive QP/);
  assert.equal(nodes['duty-value-2'].textContent, '60%');
  assert.equal(nodes['qp-status'].textContent, 'solved');

  nodes.jump.handlers.click();
  assert.equal(run('index'), 1);
  assert.equal(run('playing'), false);
  assert.equal(nodes['qp-status'].textContent, 'Contact · coast');
  assert.equal(nodes['duty-value-0'].textContent, '—');
  nodes.play.handlers.click(); run('animate(100); animate(120)');
  assert.equal(run('index'), 2);
  assert.equal(run('playing'), false, 'Playback must stop at the final state');
  assert.equal(nodes.action.textContent, 'Flight ended');
  assert.equal(nodes['qp-status'].textContent, 'Flight ended');
  assert.equal(nodes.height.textContent, '0.00');
  nodes.timeline.value = '0'; nodes.timeline.handlers.input();
  assert.equal(nodes.action.textContent, 'Main engine');
  nodes.restart.handlers.click();
  assert.equal(run('index'), 0);
  run('flight.frames[0].fallback = true; render()');
  assert.equal(nodes['qp-status'].textContent, 'Fallback · coast');
  assert.equal(nodes['duty-value-2'].textContent, '—');
  run('flight.frames[0].fallback = false; flight.frames[0].slack = 0.2; render()');
  assert.equal(nodes['insight-title'].textContent, 'A margin was relaxed.');
  run('flight.frames[0].slack = .01');

  run('load()'); await reply(pending.shift(), false);
  assert.match(nodes['load-status'].textContent, /could not be loaded/);
  assert.equal(nodes.height.textContent, '—', 'Failed loads must not display old telemetry');
  assert.equal(nodes['qp-status'].textContent, '—');
  assert.equal(nodes.play.disabled, true);
  run('load()'); await reply(pending.shift());
  assert.equal(nodes.play.disabled, false);
  assert.equal(run('flight.seed'), 7);
  console.log('Replay checks passed: controller text, request races, seed retention, playback, scrubbing, and load recovery.');
})().catch(error => { console.error(error); process.exitCode = 1; });
