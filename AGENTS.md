# Repository instructions

- Use uv with the checked-in uv.lock. Keep an independent project environment.
- Do not import lunar_rl, lunar_mpc or torch; related projects are linked, not dependencies.
- Keep the frontend in plain HTML, CSS and VanillaJS without external dependencies.
- Keep recordings, replays, logs, builds and caches under ignored dist/.
- Run CONTRIBUTING.md checks before committing. For packaging/evaluator changes,
  also run a real episode. For web-data changes, regenerate and check the app.
- Preserve the observation-only control boundary and report research limitations.
- Distinguish recorded playback from browser optimization and historical from fresh results.
- Bind local servers to 127.0.0.1. Open a browser only when requested and stop
  a server when asked.
