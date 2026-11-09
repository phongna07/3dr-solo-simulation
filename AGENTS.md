# Project and architecture

This project simulates full fleets of three or five UAVs tracking a moving target
through obstacles at 30 m altitude. Docker supplies ROS 2 Jazzy
and native Gazebo Harmonic multicopters. MPC is planar; native velocity control,
an `Actuators` component and four rotor motor models provide 3D force/torque physics.

Read `README.md` first. The four guides are `docs/SIMULATION.md` (run, record,
inspect, recover, GPU), `docs/CONFIGURATION.md` (inputs and timing),
`docs/DEVELOPMENT.md` (architecture, submodule, interfaces, replay), and
`docs/DEPLOYMENT.md` (standalone simulation images and preserved build inputs).
Keep this documentation structure compact; update an existing guide rather
than adding a separate document for each feature or experiment.

## Where to work

- `external/motion_planning_MPC/`: colleague-owned Git submodule. Solver
  `mpc_problem.py` and predictive `dynamics.py` execute directly from this source.
- `src/solo_mpc/solo_mpc/core/upstream.py`: isolated upstream imports and
  actual source provenance; do not import upstream simulator/global config.
- `core/runtime/`: ROS-free `contracts.py` (`Algorithm`/`VehiclePlanner` protocols,
  `VehicleState`), `registry.py` (`algorithm:` / `run --algorithm`, one name per
  `core/algorithms/` subpackage), the generic per-UAV worker runtime `fleet.py`,
  `workers.py` and `cycles.py`.
- `core/algorithms/<name>/`: one package per algorithm exporting `ALGORITHM`, with
  its planner in `vehicle.py`. `mpc/` integrates existing MPC (`MPCVehicle`);
  `mpc/problem.py` is the local compatibility adapter, retaining inactive-neighbor
  conditioning and optional speed recovery (do not restore copied solver code), and
  `mpc/capture.py` records optimizer inputs for replay. `apf/` runs the unchanged
  upstream baseline through `upstream_robot.py` shims and `command_rollout.py`;
  `cbf_qp/` (`CBFQPVehicle`) and `cbf_qp_lag/` (lag-aware subclass) run unchanged
  upstream `cbfqp.py` through the same adapters; `pure_mpc/` (`PureMPCVehicle`) runs
  unchanged upstream `pure_mpc.py` with the shared `stop_plan.py` fallback;
  `impc_ob_rec/` (`IMPCOBRecVehicle`) runs unchanged upstream `impc_ob.py` (recover)
  with the same fallback and the fleet `broadcast`/`peers` channel; no copied
  baseline code.
- Other `solo_mpc/core/` packages: `planning/` (local grid planning, paths,
  coordination, references, target route), `perception/` (sensors, occupancy/mapping,
  target estimation) and `video/` (mission recording panels); keep ROS imports out.
  These remain local ports and do not automatically follow upstream changes.
- `src/solo_mpc/solo_mpc/`: ROS state/sensor adapters, frames, publishing,
  mission supervision, recording, and evaluation.
- `src/solo_mpc/launch/mpc.launch.py`: single simulation launch entry point.
- `src/solo_mpc/solo_mpc/flight_node.py`: one exclusive process per UAV; token-checked commands,
  body velocity/enable output, measured takeoff/landing and motor shutdown.
- `src/solo_mpc/gazebo/`: `SoloFlightSafety`, camera/gimbal/coverage plugins.
- `src/solo_mpc/models/`: vendored original Iris assets and licenses.
- `src/solo_bringup/`: fleet validation, readiness, run files, flight helpers.
- `src/solo_mpc_interfaces/`: mission service, `FlightState`, `FlightCommand`.
- `config/`, package `config/` and `scenarios/`: fleet/controller/scenario settings.
- `docker/`, `compose*.yaml`, `locks/`: runtime, deployment, pinned dependencies.
- `runs/`, `artifacts/`: generated evidence, ignored by Git; do not delete prior runs.

This repository is simulation-only. Do not add physical-drone profiles, serial
device integration, or hardware migration scaffolding. All flight and recording
runs use native Gazebo multicopters only; do not add a legacy backend or a
one-UAV migration phase. Preserve both full-fleet CLI presets.

## Submodule workflow

Initialize before building: `git submodule update --init --recursive`.
Keep the colleague's upstream files and sibling checkout unchanged. Put
simulation-specific behavior in local adapters. Change upstream source only if
the user explicitly requests upstream development; do not patch, monkey-patch
globally, or copy the solver into this project to avoid an interface mismatch.

To update to an authorized revision, fetch in the submodule and run
`./scripts/pin-mpc-upstream COMMIT_OR_REF`. The helper rejects local submodule
changes, checks out a detached revision, and refreshes
`src/solo_mpc/UPSTREAM.json`. Review the upstream diff, validate, and include
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
- Preserve `OdometryPublisher` world pose/body twist source stamps; rotate twist
  into world ENU in `frame_node`, never restamp at receipt. Setpoint sampling and
  position feedback produce world `TwistStamped` on `/uavN/flight/velocity`.
- Only the UAV's flight process owns `/uavN/cmd_vel` body `Twist` and
  `/uavN/enable` `Bool`. Keep `SoloFlightSafety` between it and native velocity
  control: independent simulation/wall expiry must descend after process death
  and disable motors only on settled ground. Observe actual `/uavN/motor_speed`
  `actuator_msgs/Actuators` and `/uavN/safety_active`, not just command state.
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
./scripts/dev run --seconds 30
```

`run` starts, flies, lands, evaluates, finalizes any video and stops the stack.

Add `--record-video` when visual behavior or the UAV FOV panels are relevant.
Target input is always ground truth; there is no camera detection. The duration limits tracking simulation time, not startup,
takeoff, or landing. Use longer runs when needed. Stop any existing simulation
before a new one. If `run` fails, inspect its cleanup output and confirm the
container stopped. Confirm landing/disarm. Use `stop --force` only
for a crashed simulation when normal cleanup cannot complete.

Inspect the actual `runs/<run-id>/` directory; `runs/latest` moves:

- `metadata/controller_sources.jsonl` and optimizer captures: actual upstream
  source hashes, pinned commit, `matches_pinned_sources`, and adapter identity.
- `logs/`, `status/startup.json`, `health.json`, `ready.json`, and component status:
  initialization, exceptions, readiness, and shutdown.
- `telemetry/mpc.jsonl`, `telemetry/truth.jsonl`, `reports/evaluation.json`: measured
  motion, all UAVs' solver outcomes, timing, tracking, geometry, and failures.
- `recorded-mission.mp4` when requested: visually inspect the relevant result;
  `run` cleanup (or a manual `stop`) finalizes it.

For backend-wide changes, rebuild and verify `scen5_1000` and
`scen1_1000`, each with the full fleet, camera input and recording.
The November 5, 2026 native migration evidence and limitations
are recorded in `docs/DEVELOPMENT.md`; earlier backend flights are not validation.
Preserve the 1920 × 1080 / 20 FPS composition, all map/state/FOV panels, source
timestamp sidecars and recording layout. Do not introduce new CPU/performance
acceptance thresholds; retain existing caps and report failures visibly.

Optimizer replay is a numerical diagnostic, not a substitute for flight evidence.
Successful command exits alone are insufficient. Keep CPU/deadline failures
visible. Run `git diff --check` before finishing. Report changes, verification
commands, run directory, observed results, and limitations. If simulation is
blocked, state the blocker and do not claim runtime success.
