# Project and architecture

This project simulates three UAVs tracking a moving target through obstacles at
3 m altitude. Docker supplies ROS 2 Jazzy, ArduCopter SITL, MAVROS, and Gazebo
Harmonic. MPC is planar; Gazebo/ArduPilot provide 3D vehicle physics.

Read `README.md` first. The four guides are `docs/SIMULATION.md` (run, record,
inspect, recover, GPU), `docs/CONFIGURATION.md` (inputs and timing),
`docs/DEVELOPMENT.md` (architecture, submodule, interfaces, replay), and
`docs/DEPLOYMENT.md` (standalone simulation images and preserved build inputs).
Keep this documentation structure compact; update an existing guide rather
than adding a separate document for each feature or experiment.

## Where to work

- `external/motion_planning_MPC/`: colleague-owned Git submodule. Solver
  `mpc_problem.py` and predictive `dynamics.py` execute directly from this source.
- `ros_ws/src/solo_mpc/solo_mpc/core/upstream.py`: isolated upstream imports and
  actual source provenance; do not import upstream simulator/global config.
- `core/mpc_problem.py`: local compatibility adapter, retaining inactive-neighbor
  conditioning and optional speed recovery. Do not restore copied solver code.
- Other `solo_mpc/core/` modules: local planning, occupancy/mapping, target
  estimation, coordination, and persistent per-UAV workers; keep ROS imports out.
  These remain local ports and do not automatically follow upstream changes.
- `ros_ws/src/solo_mpc/solo_mpc/`: ROS state/sensor adapters, frames, publishing,
  mission supervision, recording, and evaluation.
- `ros_ws/src/solo_mpc/launch/mpc.launch.py`: single simulation launch entry point.
- `ros_ws/src/solo_mpc/gazebo/`: native camera/gimbal/coverage plugins.
- `ros_ws/src/solo_bringup/`: fleet validation, readiness, run files, MAVROS/flight helpers.
- `ros_ws/src/solo_mpc_interfaces/`: mission service definition.
- `config/`, package `config/` and `scenarios/`: fleet/controller/scenario settings.
- `docker/`, `compose*.yaml`, `locks/`: runtime, deployment, pinned dependencies.
- `runs/`, `artifacts/`: generated evidence, ignored by Git; do not delete prior runs.

This repository is simulation-only. Do not add physical-drone profiles, serial
device integration, or hardware migration scaffolding. All flight and recording
runs use Gazebo and ArduCopter SITL.

## Submodule workflow

Initialize before building: `git submodule update --init --recursive`.
Keep the colleague's upstream files and sibling checkout unchanged. Put
simulation-specific behavior in local adapters. Change upstream source only if
the user explicitly requests upstream development; do not patch, monkey-patch
globally, or copy the solver into this project to avoid an interface mismatch.

To update to an authorized revision, fetch in the submodule and run
`./scripts/pin-mpc-upstream COMMIT_OR_REF`. The helper rejects local submodule
changes, checks out a detached revision, and refreshes
`ros_ws/src/solo_mpc/UPSTREAM.json`. Review the upstream diff, validate, and include
both the gitlink and metadata when committing. Preserve provenance and licenses.
Do not advance the pin just because a newer remote revision exists.

Development loads the live submodule under `/workspace`. Installed ROS packages
bundle unchanged solver/dynamics sources and license under
`share/solo_mpc/upstream`; release runtime must work without Git or a host checkout.
Keep both paths working. The upstream requirements file includes simulation and
plotting dependencies; dependency changes belong in Docker and local locks.

## Implementation rules

- Preserve measured-state control, per-UAV isolation, common-world ENU alignment,
  simulation-time semantics, source-stamped trajectories, independent setpoint
  expiry, ownership checks, and landing/cleanup. Never bypass them to pass a run.
- In development, restart the stack for existing Python/config/launch/upstream
  edits. Rebuild with `./scripts/dev build sim` for dependencies, package metadata,
  executables/interfaces, renamed modules, and native plugins. Release source
  changes always need a new image.
- Remove code only after checking imports, entry points, launch files, and CLI
  references. Shared grid/planning utilities remain necessary even where their
  original upstream simulation classes have been removed.
- Keep changes focused, update the relevant guide, and avoid generated run data,
  local `.env` files, caches, and dependency copies in commits.

## Verification

**Do not write automated tests.** Do not add unit/integration/regression/end-to-end
suites, test harnesses, or CI workflows that run tests. Verify feature changes
through representative simulator missions and recorded evidence. Documentation
and unused-file cleanup also need reference/import checks; run a mission when
active imports, packaging, or runtime behavior change.

Routine simulation verification is authorized without additional confirmation:

```bash
./scripts/dev mpc --ground-truth-target
./scripts/dev mission run --seconds 30
./scripts/dev evaluate
./scripts/dev stop
```

Add `--record-video` when visual behavior is relevant, or use default camera mode
to check perception. The duration limits tracking wall time, not startup/takeoff/
landing. Use longer runs when needed. Stop any existing simulation before a new
one. If a command fails, inspect it and explicitly complete cleanup; commands
after a failed `&&` are skipped. Confirm landing/disarm. Use `stop --force` only
for a crashed simulation when normal cleanup cannot complete.

Inspect the actual `runs/<run-id>/` directory; `runs/latest` moves:

- `metadata/controller_sources.jsonl` and optimizer captures: actual upstream
  source hashes, pinned commit, `matches_pinned_sources`, and adapter identity.
- `logs/`, `status/startup.json`, `health.json`, `ready.json`, and component status:
  initialization, exceptions, readiness, and shutdown.
- `telemetry/mpc.jsonl`, `telemetry/truth.jsonl`, `reports/evaluation.json`: measured
  motion, all UAVs' solver outcomes, timing, tracking, geometry, and failures.
- `recorded-mission.mp4` when requested: visually inspect the relevant result;
  normal `stop` finalizes it, mission completion alone does not.

Optimizer replay is a numerical diagnostic, not a substitute for flight evidence.
Successful command exits alone are insufficient. Keep CPU/deadline failures
visible. Run `git diff --check` before finishing. Report changes, verification
commands, run directory, observed results, and limitations. If simulation is
blocked, state the blocker and do not claim runtime success.
