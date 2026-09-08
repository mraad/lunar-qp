# QP Control Lab

An HTML, CSS and VanillaJS application for exploring recorded QP decisions.
No frontend packages, CDN, external fonts, API or browser optimization. Python
only compacts the existing data; a static server serves the built files.

From the repository root:

```bash
uv run python web/build.py
uv run python -m http.server 8766 --bind 127.0.0.1 --directory dist/web
```

Open http://127.0.0.1:8766/ and stop with Ctrl+C. If recordings are missing,
first run `uv run python web/record.py`. That command runs all 112 episodes on
the CPU and builds the app. It takes a few minutes depending on the machine.
Generated files remain in ignored `dist/`.

## Explore the process

- Choose normal flight or a 30% main-engine loss at two seconds.
- Switch between adaptive and fixed-model QP without changing the selected seed.
- Seeds 0–7 are development cases; seeds 300–319 are evaluation cases.
- Scrub, play, restart, change speed or jump to the fault.
- Mint shows actual motion; gold shows the fractional QP prediction.
- Engine bars show the normalized fraction requested for each action. The
  highlighted row shows the single discrete command actually sent. A pulse can
  differ from the largest current fraction because allocation retains a balance.
- Solver status, iterations, maximum slack and residual explain the accepted
  numerical plan. Model estimates reveal only information available by that step.

Contact and rejected solves have no accepted QP request, so the bars show dashes.
At the final frame, the actual final observation is shown and QP diagnostics
are cleared. A solved QP does not guarantee a successful physical landing.
The outcome comes from the recorded strict terminal checks, not a reward threshold.

`build.py` combines the eight `dist/qp/{heldout-,}{nominal,fault}-{adaptive,fixed}.json`
reports into four datasets. It validates scenario/configuration metadata, unique
seeds, frame counts and action/fraction shapes before replacing built assets.
Source hashes remain attached to each episode; building the UI does not relabel
older recordings as freshly evaluated controller results.

The explanatory horizon and fault text assumes the default 16 four-step blocks
at 50 Hz and a 30% fault at step 100. The builder rejects mismatched settings.
The evaluation table summarizes the supplied recordings; regenerate and inspect
fresh results after changing controller behavior.

## Checks

```bash
node --check web/app.js
node web/app.test.cjs
uv run python -m unittest discover -s tests -v
```

Node is only needed for the package-free regression check. It runs the actual
application logic against a small DOM stand-in: playback, switching races, seed
retention, QP bars/status, contact/fallback/terminal states and load recovery.
It does not test canvas appearance. For a visual check, inspect desktop and
narrow screens, then compare evaluation seed 304 under the fault in both modes.

See the [QP introduction](../docs/quadratic-programming.md) and
[evaluation report](../docs/qp-evaluation.md) for the formulation and limitations.
The original RL implementation is maintained separately in [lunar-rl](https://github.com/mraad/lunar-rl).
