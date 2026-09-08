# Contributing

Use `uv sync --locked` and the checked-in `uv.lock`. Runtime dependencies are
NumPy, SciPy, OSQP and Gymnasium with Box2D. Keep the package independent of the
RL and MPC repositories. Keep the frontend in plain HTML, CSS and VanillaJS.

Before committing:

```bash
uv lock --check --offline
uv run python -m unittest discover -s tests -v
node --check web/app.js
node web/app.test.cjs
git diff --check
```

For packaging or evaluator changes, run a real episode and generate a replay:

```bash
uv run lunar-qp --episodes 1 --seed 0 --thrust-scale 0.7 \
  --out dist/smoke.json --replay dist/smoke.html
```

For web-data changes, run `uv run python web/record.py` and the visual checks in
[the web guide](web/README.md). Keep recordings, logs, replays and builds under
ignored `dist/`. Do not commit environments, credentials or generated traces.

For controller changes, use matched seeds, action constraints and compute budgets
to compare adaptive/fixed models. Report strict landing results, failure types,
solver status, slack, actuator-allocation error and deadline misses. Reserve new
evaluation seeds before tuning. The controller must not read hidden simulator
state or the evaluator's fault schedule. The current fault injector changes a
process-local Gymnasium constant; run one environment per process at a time.

Preserve the distinction between QP numerical feasibility and physical safety,
and between recorded browser playback and a new optimization.
