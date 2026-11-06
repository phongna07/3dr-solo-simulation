# Multi-UAV MPC simulation

Teams of UAVs track a moving target through obstacles using ROS 2 Jazzy and
native Gazebo Harmonic multicopters in Docker. The default three-UAV scenario
uses a 50 × 30 m field and a 3 m flight altitude. Vehicle physics runs in 3D;
MPC plans horizontally, with separate altitude control. Vendored original Iris
assets in `ros_ws/src/solo_mpc/models` retain vehicle geometry and inertias;
`MulticopterVelocityControl` writes an `Actuators` component consumed by four
`MulticopterMotorModel` systems applying rotor forces and torques. Vehicles are
not moved kinematically.

The `large200_5uav` preset uses five UAVs in a 200 × 200 m field with
seeded random obstacles and a random winding target route. Select it with
`./scripts/dev mpc --preset large200_5uav --record-video`.
This preset flies at 3 m; `--preset small50_3uav` explicitly selects the
original scenario.

`--preset scen1` scales the upstream five-UAV 1000 × 1000 m layout to
200 × 200 m. All starts, 65 obstacles and 27 route waypoints retain their
relative geometry at one fifth of their original coordinates. UAVs fly at 6 m
with a 10 m-wide camera footprint, and the observer follows 52 m above the
target. Normal/sprint target speeds are 1/2.6 m/s; the UAV prediction speed
bound is 2.2 m/s. A 0.4 m grid and 30 m local planning radius retain the original
grid workload. See the configuration guide for physical-clearance exceptions
and verification limits.

The solver and predictive dynamics come directly from a pinned, unchanged
[colleague-owned repository](https://github.com/thuhangkhuat/motion_planning_MPC)
at `external/motion_planning_MPC`. This repository focuses on Gazebo simulation
and recorded missions. Local adapters handle measured state, ROS, sensors, and
flight supervision. A ROS-free `Algorithm` protocol selects the existing MPC
through `algorithm: mpc`; planning and formation remain local ports. One flight
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
./scripts/dev mpc --record-video
./scripts/dev mission run --seconds 30
./scripts/dev evaluate
./scripts/dev stop
```

Keep an existing `.env` rather than overwriting it. `mpc` waits for readiness and
leaves motors disabled; `mission run` takes off, tracks, lands, and waits for
settled ground and motor shutdown.
The time limit covers tracking in simulation seconds, excluding startup, takeoff,
and landing; omit it or use `--seconds 0` to attempt the complete route.
Failed `mpc` and `mission run` commands request landing, finalize any recorded
video, and stop the container while retaining their failure exit status.
Physics defaults to 0.2× real time to give MPC more compute time per simulation
second; recorded video retains normal motion at 20 FPS. `--seconds 30` requests
30 simulation seconds of tracking (at least 150 wall seconds at 0.2×), regardless
of the actual simulation rate. See the configuration
guide for `simulation_real_time_factor`.

The default uses camera detections and LiDAR. Add `--ground-truth-target` to
`mpc` to supply timestamped target positions directly. Add `--record-video` for
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
ros_ws/src/solo_mpc/          Solver adapter, local planning, ROS launch/nodes, Gazebo plugins
ros_ws/src/solo_mpc/models/   Vendored original Iris assets and licenses
ros_ws/src/solo_bringup/      Fleet validation, readiness, run files, flight helpers
ros_ws/src/solo_mpc_interfaces/ Mission and owner-checked flight interfaces
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

The three current full-fleet presets completed bounded native missions with
camera input, LiDAR mapping and recording on November 5, 2026. Every vehicle landed and stopped
its rotors. An additional all-flight-process pause confirmed independent Gazebo
watchdog landing; that deliberate fault remains a failed supervised mission.
See [Development](docs/DEVELOPMENT.md#native-migration-verification) for exact
run IDs, visible solver/deadline failures, video inspection and limitations.
Installed runtime needs neither Git nor a host checkout; see
[Deployment](docs/DEPLOYMENT.md).
