# Standalone package validation

Validated on 2026-09-08 using Python 3.12.12 and the checked-in uv lockfile.
This is a packaging/extraction regression run, not a new held-out research set.
Seeds 0–7 and 300–319 were already evaluated in the original research worktree.
The controller equations, tuning and solver settings were retained. Changes to
the controller module are its provenance note and evaluator helper imports.

The independent `lunar_qp` package uses its own environment helpers, Git history
and virtual environment. Source imports contain neither sibling package nor
PyTorch. No RL checkpoint, generated recording or original Git history is shipped.

## Checks performed

- Locked dependency installation and offline lock consistency passed.
- All nine Python controller/environment tests passed.
- JavaScript syntax and the package-free application regression check passed.
- The installed `lunar-qp` CLI completed fault seed 0 and produced JSON and a
  self-contained HTML replay; its strict landing check passed.
- `uv run python web/record.py` regenerated all 112 episodes and built the app.
- A temporary localhost server returned HTTP 200 for the index, stylesheet,
  script and all four valid JSON datasets. It was stopped after the smoke test.
- Wheel and source distribution builds passed. The wheel contains the replay
  template, license and NOTICE; neither archive includes the environment or
  generated `dist/` data.

The JavaScript test uses a DOM stand-in and does not assess canvas appearance.

## Reproduced landing results

Strict success requires the final landing checks and both rendered feet inside
the pad; reward alone is insufficient. The fault is a 30% main-engine reduction
at step 100. Each row uses the default horizon 16, hold 4 and 4,000 solver limit.

| Seeds | Scenario | Model | Passed | Fallback steps |
|---|---|---|---:|---:|
| 0–7 | nominal | adaptive | 8/8 | 0 |
| 0–7 | nominal | fixed | 8/8 | 0 |
| 0–7 | fault | adaptive | 8/8 | 0 |
| 0–7 | fault | fixed | 8/8 | 0 |
| 300–319 | nominal | adaptive | 20/20 | 0 |
| 300–319 | nominal | fixed | 20/20 | 0 |
| 300–319 | fault | adaptive | 15/20 | 0 |
| 300–319 | fault | fixed | 16/20 | 0 |

Controller SHA-256: `f79df07d8afa7aada4653656d6d05fdf1a6506062ce5961f1df03e4e58e807bb`.

The recorded pass counts match the historical evaluation. They do not establish
an advantage from adaptation or a safety guarantee. Retained failures are
visible in the application. Detailed timings, final states and diagnostics are
written to `dist/qp/`; timings depend on hardware and concurrent load.

## Reproduce

```bash
uv sync --locked
uv lock --check --offline
uv run python -m unittest discover -s tests -v
node --check web/app.js
node web/app.test.cjs
uv run lunar-qp --episodes 1 --seed 0 --thrust-scale 0.7 \
  --out dist/smoke.json --replay dist/smoke.html
uv run python web/record.py
uv build --no-sources --out-dir dist/packages
```

See the [historical evaluation](qp-evaluation.md), [QP introduction](quadratic-programming.md)
and [web instructions](../web/README.md) for assumptions and further checks.
