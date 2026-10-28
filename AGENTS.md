# Project overview

This project simulates multiple UAVs tracking a moving target through obstacles
using model predictive control (MPC). The default scenario has three Iris UAVs
in a 50 × 30 m field flying at 3 m. Docker supplies ROS 2 Jazzy, Gazebo Harmonic,
ArduCopter SITL, and MAVROS; the host needs Docker Compose and Python 3.

Simulation is the primary development environment. The hardware profile and
standalone `hardware_mpc/` workspace need additional integration before physical
flight. Successful simulation does not establish hardware flight readiness.

## Where to work

- `scripts/dev`: primary host CLI for builds, simulation, missions, and inspection.
- `ros_ws/src/solo_bringup/`: fleet configuration, launch, readiness, and flight helpers.
- `ros_ws/src/solo_mpc/solo_mpc/core/`: numerical MPC, planning, mapping, tracking,
  and persistent per-UAV workers; keep this code independent of ROS.
- `ros_ws/src/solo_mpc/solo_mpc/`: ROS adapters, frames, perception, setpoint
  publishing, mission supervision, recording, and evaluation.
- `ros_ws/src/solo_mpc/gazebo/`: native world-stabilized camera plugin.
- `ros_ws/src/solo_mpc_interfaces/`: mission service definition.
- `config/`, `ros_ws/src/solo_mpc/config/`, and
  `ros_ws/src/solo_mpc/scenarios/`: fleet, controller, and scenario settings.
- `docker/`, `compose*.yaml`, and `locks/`: runtime, deployment, and pinned inputs.
- `hardware_mpc/`: standalone hardware integration workspace; consider whether
  shared controller changes also apply there.
- `runs/` and `artifacts/`: generated evidence, ignored by Git.

Read `README.md` and `docs/GETTING_STARTED.md` first. Consult
`docs/DEVELOPMENT.md` for architecture and ROS interfaces,
`docs/CONFIGURATION.md` for settings, and `docs/OPERATIONS.md` for recovery and
recording details.

## Implementation instructions

- Keep changes focused and update documentation when commands or behavior change.
- Preserve measured-state control, per-UAV isolation, common-world ENU alignment,
  simulation-time semantics, independent setpoint expiry, and mission cleanup.
- Do not bypass readiness, watchdogs, ownership checks, or landing to make a run
  appear successful. Investigate the recorded failure instead.
- In development, the checkout is mounted at `/workspace` in the container.
  Restart the simulation to apply existing Python, launch, or configuration edits.
  Rebuild with `./scripts/dev build sim` for native plugin changes, dependencies,
  new executables, package metadata, or interfaces.
- Keep dependency changes in the Dockerfile and locks. Preserve upstream
  provenance and licenses. Do not commit generated run data or local `.env` files.

## Verification: run the project and inspect evidence

**Do not write automated tests at all.** Do not add unit, integration, regression,
or end-to-end tests, test suites, test harnesses, or CI workflows that run tests.
Verify feature implementations and fixes by running representative simulator
missions and inspecting logs, telemetry, evaluation reports, or recorded video.

Agents can run commands like this from the repository root without asking for
additional permission for routine simulation verification:

```bash
./scripts/dev mpc --record-video && ./scripts/dev mission run --seconds 20 && ./scripts/dev stop
```

Use similar commands suited to the change, such as adding
`--ground-truth-target` or selecting a different fleet, scenario, or settings file.
`--seconds 20` limits tracking in wall-clock seconds; startup, takeoff, and landing
take additional time. Startup may take several minutes with software rendering.
Use a longer mission when the feature requires more time to observe.

For numerical evidence, run evaluation after the mission lands and before
stopping the container:

```bash
./scripts/dev mpc --record-video
./scripts/dev mission run --seconds 20
./scripts/dev evaluate
./scripts/dev stop
```

Inspect the specific `runs/<run-id>/` directory; `runs/latest` changes with each
launch. Check the evidence relevant to the feature:

- `logs/`: process errors, warnings, and shutdown behavior.
- `status/startup.json`, `status/health.json`, `status/ready.json`, and
  `status/components/`: initialization and readiness.
- `telemetry/mpc.jsonl` and `telemetry/truth.jsonl`: measured tracking, controller
  outcomes, timing, and independent vehicle positions.
- `reports/evaluation.json`: tracking, geometry, compute, and simulation speed.
- `recorded-mission.mp4`: visually inspect the overhead view, occupancy map,
  drone state instruments, and UAV camera feeds for the intended behavior.
  Normal `./scripts/dev stop` finalizes the recording; mission completion alone
  does not close it.

While running, use `./scripts/dev status`, `./scripts/dev mission status`,
`./scripts/dev logs`, `./scripts/dev doctor`, or `./scripts/dev shell` as needed.
A successful command exit alone is insufficient: confirm the feature's observable
result and check for relevant failures in the evidence.

If a command in an `&&` chain fails, later commands are skipped. Inspect the
failure and explicitly run `./scripts/dev mission stop` or `./scripts/dev stop`
as needed. Confirm landing and disarm; use `stop --force` only for a crashed
simulation when normal cleanup cannot complete. Stop an existing simulation
before starting a new one.

Run `git diff --check` before finishing. Report what changed, the commands run,
the run directory and evidence reviewed, the observed result, and any limitations.
If simulation could not run, state the blocker and do not claim runtime success.
