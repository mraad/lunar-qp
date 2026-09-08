# QP evaluation

These measurements were recorded in the original QP research worktree before
standalone extraction. The controller equations and settings were preserved.
The original raw traces are not bundled; the commands below regenerate them.
See [standalone validation](standalone-validation.md) for the package checks.

These are measurements of the additional QP controller, not the original RL
policy. The RL trainer, viewer, network source and tracked checkpoints were
left unchanged in the quadratic worktree.

## Protocol

Measured on 2026-09-08 using Python 3.12.12, NumPy 2.5.2, SciPy 1.18.1,
OSQP 1.1.3 and Gymnasium 1.3.0 on the local Apple-silicon CPU. Each episode
has its own fresh nominal model and pulse-allocation balance. The simulator
uses the discrete four-action interface and the same randomized off-pad pose
distribution as the RL viewer.

Development seeds 0–7 informed the descent reference, QP formulation and solver
settings. The final configuration uses 16 four-step prediction blocks (1.28 s)
and a fixed OSQP penalty parameter. Seeds 300–319 were then evaluated without
further controller tuning. These finite samples do not establish universal
reliability. Use a new evaluation set if these failures guide a later change.

The fault is a 30% main-engine power reduction immediately before action 100.
The controller receives neither its time nor its multiplier. A pass requires
termination without a crash or time limit, a settled body, both legs in contact,
and both rendered feet between the pad flags. Merely receiving the simulator's
final +100 reward is insufficient.

## Landing results

| Seeds | Scenario | Controller | Strict passes | Mean return | Fallback steps |
|---|---|---|---:|---:|---:|
| 0–7 development | Normal | Adaptive QP | 8/8 | +267.3 | 0 |
| 0–7 development | Engine fault | Adaptive QP | 8/8 | +257.9 | 0 |
| 0–7 development | Normal | Fixed QP | 8/8 | +269.6 | 0 |
| 0–7 development | Engine fault | Fixed QP | 8/8 | +282.5 | 0 |
| 300–319 evaluation | Normal | Adaptive QP | 20/20 | +278.8 | 0 |
| 300–319 evaluation | Engine fault | Adaptive QP | 15/20 | +239.4 | 0 |
| 300–319 evaluation | Normal | Fixed QP | 20/20 | +282.8 | 0 |
| 300–319 evaluation | Engine fault | Fixed QP | 16/20 | +247.7 | 0 |

## Evaluation failures

**heldout-nominal-adaptive:** no failed terminal checks in this sample.

**heldout-fault-adaptive:** seed 304 (crash, foot margin -0.567); seed 305 (off-pad settled landing, foot margin -1.107); seed 307 (crash, foot margin 1.137); seed 308 (crash, foot margin 1.298); seed 310 (off-pad settled landing, foot margin -0.172).

**heldout-nominal-fixed:** no failed terminal checks in this sample.

**heldout-fault-fixed:** seed 303 (crash, foot margin 0.692); seed 305 (crash, foot margin -0.454); seed 306 (crash, foot margin 0.700); seed 308 (crash, foot margin -2.774).

The QP can satisfy its mathematical constraints while the real discrete
trajectory fails. The locally linear model, fractional-to-discrete actuator
conversion and softened approach limits remain approximations. Positive slack
means a planned margin was relaxed. A successful solve is not a safe landing.

## Planning time

Times cover `act()`: QP construction, solve, validation, prediction reconstruction
and pulse allocation. They exclude model updates and simulator stepping.
Recorded runs overlap other CPU work; these are observed timings, not isolated
hardware benchmarks. The simulator waits for each command and does not apply
the physical consequences of deadline misses.

| Evaluation | Median ms | P95 ms | P99 ms | Maximum ms | Calls over 20 ms | Maximum slack |
|---|---:|---:|---:|---:|---:|---:|
| nominal-adaptive | 6.31 | 8.02 | 9.48 | 51.11 | 6 | 0.436 |
| fault-adaptive | 5.24 | 7.73 | 9.36 | 52.28 | 4 | 1.924 |
| nominal-fixed | 6.16 | 7.77 | 9.04 | 53.26 | 4 | 0.025 |
| fault-fixed | 5.40 | 7.91 | 9.27 | 45.35 | 3 | 1.291 |

## Reproduce and inspect

```bash
uv sync --locked
uv run lunar-qp --seed 300 --episodes 20 --out dist/qp/heldout-nominal-adaptive.json
uv run lunar-qp --seed 300 --episodes 20 --thrust-scale 0.7 --out dist/qp/heldout-fault-adaptive.json
uv run lunar-qp --seed 300 --episodes 20 --fixed-model --out dist/qp/heldout-nominal-fixed.json
uv run lunar-qp --seed 300 --episodes 20 --fixed-model --thrust-scale 0.7 --out dist/qp/heldout-fault-fixed.json
```

For a focused replay, set `--episodes 1 --seed <seed>` and add
`--replay dist/qp/inspection.html`. Generated JSON and HTML stay in ignored
`dist/qp/`. The replay opens directly without a server. The traces contain
observations, actual discrete actions, relaxed plans/predictions, online model
estimates, final states, solver status, constraint violation, slack and timing.

The seven focused QP regression tests pass, including finite-difference model
derivatives, positive-definite cost, linear feasibility, prediction reconstruction,
pulse counts, online estimation and explicit fallback handling. The original
`lunar_rl.nets` self-check, offline lock check and both tracked checkpoint SHA-256
checks also pass. Replay JavaScript passes `node --check`; no browser rendering
test was performed for the QP replay.

These results should not be compared directly with previous beam-search MPC
scores as a claim about solver superiority: the references, costs, discretization
and evaluation seeds differ. Match those choices and report actuator-allocation
error and deadline misses before making such a claim. MIQP and a validated
recovery strategy remain research work, not features implemented here.

Controller source SHA-256 for these runs:

```text
8c23b271f98646f9ba3234feda6ec16af4d61ea155e4d224326225352eff5709
```

The hash identifies `qp.py`; preserve the lockfile and repository version too,
because simulator helpers and dependencies also influence results. Seeded runs
and timing are not guaranteed bitwise identical on other platforms.

The subsequent web-app change simplified repeated array conversions and solver
result validation without changing the cost, constraints or controller settings.
Old/new implementations produced identical actions, predicted states, solver
diagnostics and engine estimates throughout the 402-step seed-0 fault recording.
The seven QP regressions also pass. The 112 recordings above retain their original
source hash; this equivalence check is not a fresh evaluation of all 112 flights.
