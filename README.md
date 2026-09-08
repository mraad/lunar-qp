# Lunar QP

**From fractions to flight.** A quadratic-programming controller for LunarLander,
with online physics estimation and an elegant, dependency-free VanillaJS app.

At each observation, OSQP chooses a short plan of engine-duty fractions. A pulse
allocator converts the first request into one legal discrete engine command.
The controller observes the result, updates its engine model, and solves again.
This is QP-based Model Predictive Control (MPC): QP solves the plan; MPC repeats
the planning loop. It is a convex relaxation with pulse allocation, not MIQP.

This repository has its own Git history, package and uv environment. No RL
checkpoint, PyTorch installation, or sibling repository is required. The
original RL project remains available independently.

## Showcase

[![QP MPC landing after an engine-power loss](docs/showcase.gif)](docs/showcase.mp4)

This is an actual QP-controller run. At two seconds the main engine loses 30%
of its power; OSQP continues producing fractional plans and the pulse allocator
turns each first-step request into a legal discrete action. Click the preview
for the MP4. Recreate both files with `uv run python scripts/record_showcase.py`;
encoding requires `ffmpeg`.

## Related projects

Three complementary LunarLander control experiments:

| Project | Approach |
|---|---|
| [Lunar RL](https://github.com/mraad/lunar-rl) | Transformer PPO policy trained through reinforcement learning |
| [Lunar MPC](https://github.com/mraad/lunar-mpc) | Adaptive physics model with discrete beam-search predictive control |
| [Lunar QP](https://github.com/mraad/lunar-qp) | Convex quadratic-programming MPC with discrete engine-pulse allocation |

Each project documents its own setup, assumptions and evaluation. MPC is the
repeated planning loop; QP is one way to solve the plan within that loop.

## Quick start

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) and Git.
Python 3.12 is selected by `.python-version`; uv can provision it if necessary.
Box2D may need a C/C++ compiler; install Xcode Command Line Tools on macOS if
required. SWIG is supplied by the locked dependencies/build configuration.

```bash
git clone https://github.com/mraad/lunar-qp.git
cd lunar-qp
uv sync --locked
uv run python web/record.py
uv run python -m http.server 8766 --bind 127.0.0.1 --directory dist/web
```

Open **http://127.0.0.1:8766/**. Stop the server with **Ctrl+C**. Recording all
112 flights takes a few minutes on the CPU. Generated recordings and web assets
stay under ignored `dist/`; no recordings are needed from another repository.
After recording, `uv run python web/build.py` rebuilds the app without simulating
again. If port 8766 is occupied, choose another port in the server command.

The browser uses only HTML, CSS, VanillaJS and local JSON: no npm, frontend
framework, external fonts, CDN or API. It replays actual recorded OSQP decisions;
changing a selection does not run a new optimization in the browser.

## Explore the QP Control Lab

- Choose normal flight or a 30% main-engine loss at two seconds.
- Compare adaptive and fixed-model QP on development and evaluation seeds.
- Play, pause, scrub, change speed or jump to the fault.
- Compare the fractional predicted path with actual motion.
- Inspect engine-request fractions, the executed pulse, solver status,
  iterations, constraint residuals, slack and the evolving engine estimate.

The app includes 8 development and 20 evaluation starts for each of four
scenario/controller combinations. Its evaluation table is built from the
recordings. Failed landings are retained for inspection. Contact, fallback and
terminal states are distinguished from an accepted QP plan.

## Run a controller experiment

```bash
uv run lunar-qp --episodes 8 --seed 0 \
  --out dist/qp/nominal-adaptive.json --replay dist/qp/nominal-adaptive.html
uv run lunar-qp --episodes 8 --seed 0 --thrust-scale 0.7 \
  --out dist/qp/fault-adaptive.json --replay dist/qp/fault-adaptive.html
uv run lunar-qp --episodes 8 --seed 0 --thrust-scale 0.7 --fixed-model \
  --out dist/qp/fault-fixed.json
uv run lunar-qp --help
```

Single-file HTML replays open directly without a server. The equivalent module
command is `uv run python -m lunar_qp.qp`. Defaults are 16 four-step prediction
blocks (1.28 seconds), a new action every 0.02 simulated seconds, and at most
4,000 solver iterations. No GPU or policy training is required.

The adaptive controller estimates engine effectiveness from observed motion.
`--fixed-model` disables estimation updates while keeping replanning active.
Both controllers use the same four discrete actions and receive neither the
scheduled fault time nor its true multiplier. The evaluator can inspect
simulator truth to score the landing and draw the replay; the controller cannot.

## Documentation

| Document | Contents |
|---|---|
| [QP introduction](docs/quadratic-programming.md) | Beginner explanation, equations, condensation, constraints, adaptation and pulse allocation |
| [Research evaluation](docs/qp-evaluation.md) | Historical experiment protocol, landing results, failures and timing |
| [Standalone validation](docs/standalone-validation.md) | Checks performed for the independent package |
| [Web application](web/README.md) | Build, recording format, controls and browser checks |
| [Contributing](CONTRIBUTING.md) | Environment and required source/research checks |
| [Research TODO](TODO.md) | Recovery, discrete-allocation error, MIQP and uncertainty work |

```text
src/lunar_qp/qp.py           model, QP controller, evaluator and CLI
src/lunar_qp/environment.py  evaluator-only starting pose and terrain helpers
src/lunar_qp/qp_replay.html  self-contained replay template
web/                        static app, builder, recorder and JavaScript check
tests/                      controller and environment regressions
docs/                       introduction, research results and validation
```

## Research limits

Pre-extraction evaluation on seeds 300–319 passed 20/20 normal flights for both
controllers. Under the engine fault, adaptive QP passed 15/20 and fixed QP 16/20.
These results do not show an advantage from adaptation. Failures included crashes
and off-pad landings. All development starts had passed, illustrating why a
separate evaluation matters.

The fractional QP plan and real discrete engine pulses have different dynamics.
Local linearization, softened approach margins and approximate contact handling
remain limitations. A solved QP is not a safe-landing certificate. Planning-time
outliers exceed the 20 ms simulated control interval; the simulator waits for
commands and does not model the physical effects of missed deadlines.

## Checks

```bash
uv lock --check --offline
uv run python -m unittest discover -s tests -v
node --check web/app.js
node web/app.test.cjs
```

Node is used only for the JavaScript regression check and needs no packages.
See [Contributing](CONTRIBUTING.md) for real-episode and research validation.

## License

[Apache-2.0](LICENSE). Copyright 2026 mraad. [NOTICE](NOTICE) records the shared
source origins. The independent package includes no RL weights or prior Git history.
