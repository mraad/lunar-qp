# Quadratic programming research TODO

- [x] Create a dedicated quadratic-programming worktree.
- [x] Add a convex QP planner with local affine dynamics and condensed states.
- [x] Bound engine duties and add explicit approach slack penalties.
- [x] Convert fractional duties to legal discrete actions and retain actual feedback.
- [x] Support adaptive and fixed-model variants with an unannounced engine fault.
- [x] Record solver diagnostics, fallback use, plans and strict terminal checks.
- [x] Add a self-contained QP replay and introductory implementation guide.
- [x] Test model derivatives, convexity, feasibility, pulse allocation and failures.
- [x] Run 32 development episodes and 80 fresh-seed evaluation episodes; document
  strict landing counts, failures, solver diagnostics and planning latency.
- [x] Add a dependency-free VanillaJS QP Control Lab with all recorded flights,
  fractional requests, actual pulses, solver diagnostics and MPC-style visuals.
- [x] Simplify solver-result validation and repeated array conversions; verify
  identical decisions, predictions and estimates across a recorded fault flight.
- [ ] Investigate why adaptive QP passed 15/20 fault cases while fixed QP passed
  16/20; use new evaluation seeds after investigating the current failures.
- [ ] Validate a recovery controller and enforce full-cycle execution deadlines.
- [ ] Quantify prediction error caused by fractional-to-discrete action allocation.
- [ ] Compare against beam-search MPC with matched costs, horizon and compute budget.
- [ ] Evaluate MIQP with explicit binary engine choices.
- [ ] Add noise, observation delay, terrain sensing and uncertainty bounds.
- [ ] Reserve fresh seeds after using evaluation failures to guide any tuning.
