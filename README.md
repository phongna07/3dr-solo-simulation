# Multi-UAV MPC simulation

Three UAVs track a moving target through obstacles using ROS 2 Jazzy, ArduCopter
4.5.3 SITL, MAVROS 2.15.1, and Gazebo Harmonic in Docker. The default scenario
uses a 50 × 30 m field and a 3 m flight altitude. Vehicle physics runs in 3D;
MPC plans horizontally, with separate altitude control.

The solver and predictive dynamics come directly from a pinned, unchanged
[colleague-owned repository](https://github.com/thuhangkhuat/motion_planning_MPC)
at `external/motion_planning_MPC`. Local adapters handle measured state, ROS,
sensors, and flight supervision. Planning and formation remain local ports.

## Quick start

The host needs Docker Compose and Python 3; ROS and Gazebo run inside the image.
Run these commands from this repository's root:

```bash
git submodule update --init --recursive
test -e .env || cp .env.example .env
./scripts/dev build sim
./scripts/dev doctor
./scripts/dev mpc
./scripts/dev mission run --seconds 30
./scripts/dev evaluate
./scripts/dev stop
```

Keep an existing `.env` rather than overwriting it. `mpc` waits for readiness and
leaves UAVs disarmed; `mission run` takes off, tracks, lands, and waits for disarm.
The time limit covers tracking in wall seconds, excluding startup and landing.
If a command fails, inspect it and run cleanup explicitly.

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
| [Deployment](docs/DEPLOYMENT.md) | Installed images, dependency preservation, and the optional MAVROS companion profile |

## Layout

```text
external/motion_planning_MPC/  Unchanged upstream solver submodule
ros_ws/src/solo_mpc/          Solver adapter, local planning, ROS nodes, Gazebo camera plugin
ros_ws/src/solo_bringup/      Fleet launch, readiness, frame/flight helpers
ros_ws/src/solo_mpc_interfaces/ Mission service
config/                      Fleet and companion configuration
scripts/dev                  Primary host CLI
scripts/pin-mpc-upstream      Select an upstream revision and refresh provenance
docker/, compose*.yaml, locks/ Runtime images and pinned dependencies
docs/                        Four task-oriented guides
runs/, artifacts/            Generated evidence, ignored by Git
AGENTS.md                    Contributor instructions
```

Each launch creates `runs/<run-id>/`; `runs/latest` changes on the next launch.
Preserve the actual run directory, source hashes, resolved configuration, and
image identity when comparing experiments. Simulation is the supported flight
environment; the optional companion profile starts MAVROS/readiness only.
