# Multi-UAV MPC simulation

Teams of UAVs track a moving target through obstacles using ROS 2 Jazzy and
native Gazebo Harmonic multicopters in Docker. The default three-UAV scenario
uses a 1000 × 1000 m field and a 30 m flight altitude. Vehicle physics runs in 3D;
MPC plans horizontally, with separate altitude control. Vendored original Iris
assets in `src/solo_mpc/models` retain vehicle geometry and inertias;
`MulticopterVelocityControl` writes an `Actuators` component consumed by four
`MulticopterMotorModel` systems applying rotor forces and torques. Vehicles are
not moved kinematically.

`--preset scen5_1000` explicitly selects this scenario, upstream `scen5` with its
three active UAVs, seeded obstacle map and leader hand-off sprints.

`--preset scen1_1000` runs the upstream five-UAV 1000 × 1000 m scenario unscaled
at 30 m; see the configuration guide for its few simulation-only differences.

The solver and predictive dynamics come directly from a pinned, unchanged
[colleague-owned repository](https://github.com/thuhangkhuat/motion_planning_MPC)
at `external/motion_planning_MPC`. This repository focuses on Gazebo simulation
and recorded missions. Local adapters handle measured state, ROS, sensors, and
flight supervision. A ROS-free algorithm registry selects the existing MPC
(`algorithm: mpc`) or the upstream APF (`apf`), CBF-QP (`cbf_qp`,
`cbf_qp_lag`), plain MPC (`pure_mpc`) and IMPC-OB with recovery
(`impc_ob_rec`) baselines, also via
`run --algorithm NAME`; planning and formation remain local ports. One flight
process per UAV owns body velocity/enable commands and token-checked flight
services. A Gazebo safety plugin independently expires commands, descends, and
disables motors only on settled ground, including after process death.

## Quick start

The host needs Docker Compose and Python 3; ROS and Gazebo run inside the image.
Run these commands from this repository's root:

```bash
git submodule update --init --recursive
test -e .env || cp .env.example .env
./scripts/dev build sim
./scripts/dev doctor
./scripts/dev run --seconds 30 --record-video
```

Keep an existing `.env` rather than overwriting it. `run` is the single
simulation command: it starts a fresh stack, waits for readiness, takes off,
tracks, lands, waits for settled ground and motor shutdown, finalizes any
recorded video, stops the container, and writes `reports/evaluation.json`.
The time limit covers tracking in simulation seconds, excluding startup, takeoff,
and landing; omit it or use `--seconds 0` to attempt the complete route.
Success, failure and Ctrl+C all take the same cleanup path; a failed or
cancelled run retains its nonzero exit status.
Physics defaults to 0.2× real time to give MPC more compute time per simulation
second; recorded video retains normal motion at 20 FPS. `--seconds 30` requests
30 simulation seconds of tracking (at least 150 wall seconds at 0.2×), regardless
of the actual simulation rate. See the configuration
guide for `simulation_real_time_factor`.

The controller uses exact timestamped ground-truth target positions and every
UAV's LiDAR; there is no camera target detection. Add `--record-video` for
a synchronized overhead/map/state/UAV-camera recording, or `--gui` for a viewer.
The server otherwise runs headlessly. See the simulation guide for GPU setup.

## Guides

| Guide | Contents |
|---|---|
| [Simulation](docs/SIMULATION.md) | Run, record, inspect, recover, and configure GPU/display access |
| [Configuration](docs/CONFIGURATION.md) | Fleet, scenarios, sensors, parameters, and timeouts |
| [Development](docs/DEVELOPMENT.md) | Submodule updates, adapters, ROS interfaces, replay, and verification |
| [Deployment](docs/DEPLOYMENT.md) | Standalone simulation images and dependency preservation |

## Layout

```text
external/motion_planning_MPC/  Unchanged upstream solver submodule
src/solo_mpc/                Solver adapter, local planning, ROS launch/nodes, Gazebo plugins
src/solo_mpc/models/         Vendored original Iris assets and licenses
src/solo_bringup/            Fleet validation, readiness, run files, flight helpers
src/solo_mpc_interfaces/     Mission and owner-checked flight interfaces
config/                      Simulation fleet configuration
scripts/dev                  Primary host CLI
scripts/pin-mpc-upstream      Select an upstream revision and refresh provenance
docker/, compose*.yaml, locks/ Runtime images and pinned dependencies
docs/                        Four task-oriented guides
runs/, artifacts/            Generated evidence, ignored by Git
AGENTS.md                    Contributor instructions
```

Each launch creates `runs/<run-id>/`; `runs/latest` changes on the next launch.
Preserve the actual run directory, source hashes, resolved configuration, and
image identity when comparing experiments. All flight runs use native Gazebo
multicopter physics, with no legacy backend or one-UAV migration phase. The
project has no physical-drone profile, serial-device
configuration, or hardware migration workspace.

The full-fleet presets current at the time completed bounded native missions with
camera input, LiDAR mapping and recording on November 5, 2026. Every vehicle landed and stopped
its rotors. An additional all-flight-process pause confirmed independent Gazebo
watchdog landing; that deliberate fault remains a failed supervised mission.
See [Development](docs/DEVELOPMENT.md#native-migration-verification) for exact
run IDs, visible solver/deadline failures, video inspection and limitations.
Installed runtime needs neither Git nor a host checkout; see
[Deployment](docs/DEPLOYMENT.md).
