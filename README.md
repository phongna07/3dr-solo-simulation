# 3DR Solo: multi-UAV target tracking

This project runs a fleet of simulated UAVs that track a moving target and avoid
obstacles using model predictive control (MPC). The default mission uses three
Iris vehicles, horizontal LiDAR scans, and downward cameras in a 50 × 10 m field.
Vehicle state comes from MAVROS telemetry; the cameras detect a magenta target
marker. The controller plans horizontal motion at a fixed flight altitude.
Each UAV's mapping, planning, and MPC runs in its own persistent process, with
shared formation coordination and a separate camera detector process per UAV.

Docker supplies **ROS 2 Jazzy, ArduCopter 4.5.3, MAVROS 2.15.1, and Gazebo
Harmonic**. Python launch files manage the processes, and `scripts/dev` provides
the command-line workflow. You do not need ROS or Gazebo installed on the host.

Simulation is the working environment. The Raspberry Pi / 3DR Solo companion
profile currently provides MAVROS and readiness monitoring. Physical sensors and
hardware MPC flight still require integration and validation; see the
[hardware migration guide](docs/HARDWARE.md).

## First run

Use a Linux host or Ubuntu under Windows 11 WSL2 with Docker and Docker Compose
available. The host also needs Python 3 for the readiness helper. On WSL2, enable
Docker Desktop integration for your Ubuntu distribution and keep the repository
in the Linux filesystem, such as `~/3dr-solo`. The first image build downloads
dependencies and compiles ArduPilot, MAVROS, and the Gazebo plugin.

Run these commands from the repository root. Create `.env` once; keep an existing
file if you have already configured the project.

```bash
cp .env.example .env
./scripts/dev doctor
./scripts/dev build sim
./scripts/dev test
./scripts/dev mpc
```

`mpc` starts the simulator, one software-in-the-loop (SITL) flight controller and
MAVROS instance per UAV, sensor bridges, and the MPC nodes. MAVROS connects ROS
to each flight controller (FCU). The command waits for connected FCUs and fresh
telemetry/pose. **Starting the stack does not arm the vehicles.**
While waiting, the terminal shows launched processes, initialized MPC nodes,
and each UAV's remaining checks: MAVROS plugin configuration, FCU connection,
telemetry acknowledgements, and fresh local poses. Progress updates immediately
when a check changes and every 15 seconds during longer waits. `RUNNING` means
the process has started; `READY` nodes have completed initialization. Simulation
readiness still refers to FCU telemetry and poses; mission inputs and frame
alignment may still be pending. The same progress is available with
`./scripts/dev wait`, and full process output with `./scripts/dev logs`.
Camera and LiDAR rendering is required even with the GUI closed; use the
[WSL and GPU guide](docs/WSL_GPU.md) if rendering fails or runs slowly.

To record every UAV's downward FOV camera, start with
`./scripts/dev mpc --record-cameras` (combine with `--gui` if desired). Videos
are saved on the host in `runs/<run-id>/cameras/uavN.avi`. Recording continues
until `./scripts/dev stop`, which closes the video files. See
[camera recording](docs/OPERATIONS.md#record-uav-camera-video) for details.

Once the stack is ready:

```bash
./scripts/dev mission run       # Take off, track the target, then LAND and disarm
./scripts/dev evaluate          # Write metrics for the latest mission
./scripts/dev stop              # Release control, confirm landing, stop the container
```

Mission startup arms and launches the UAVs concurrently. Tracking and target
motion begin only after every UAV holds within 0.5 m of the flight altitude,
with vertical speed at most 0.2 m/s, for two seconds of fresh pose telemetry.

For a short experiment, use `./scripts/dev mission run --seconds 20`. This limits
tracking to 20 wall-clock seconds after takeoff and controller initialization.
Ctrl+C in `mission run` attempts to release control and land the fleet. Check its
output for successful cleanup.

In another terminal, use `./scripts/dev gui` to open Gazebo, `./scripts/dev status`
for fleet readiness, `./scripts/dev mission status` for controller diagnostics,
or `./scripts/dev logs` to follow logs. Closing the GUI leaves the server running.
The [operations guide](docs/OPERATIONS.md) covers mission control and recovery.

## Read next

| Guide | Use it to |
|---|---|
| [Configuration](docs/CONFIGURATION.md) | Change fleet size, routes, sensors, and controller settings |
| [Operations](docs/OPERATIONS.md) | Supervise missions, inspect ROS data, collect logs, and troubleshoot |
| [WSL and GPU](docs/WSL_GPU.md) | Configure the GUI and rendering on WSL or native Linux |
| [Development](docs/DEVELOPMENT.md) | Understand the architecture, ROS interfaces, and contribution workflow |
| [Reproducibility](docs/REPRODUCIBILITY.md) | Understand dependency locks and share the same image across machines |
| [Hardware migration](docs/HARDWARE.md) | Move toward Raspberry Pi 4, 3DR Solo, LD19, and GoPro deployment |
| [Validation](docs/VALIDATION.md) | Review measured simulation results and what remains unverified |

## Repository layout

```text
config/                  Fleet and hardware link configuration
docker/                  Image build and runtime entrypoint
locks/                   Source versions and Python dependency locks
ros_ws/src/solo_bringup/  Gazebo/SITL/MAVROS launch, readiness, internal flight helpers
ros_ws/src/solo_mpc/      Planner, solver, sensor adapters, mission CLI, evaluation
scripts/                 Development CLI and host/container health helpers
tests/                   Configuration, controller, ROS adapter, and CLI tests
docs/                    Task-specific guides
runs/                    Generated experiment data (Git-ignored)
artifacts/               Exported manifests and validation evidence (Git-ignored)
```

Each launch creates a fresh `runs/<run-id>/`; `runs/latest` points to the latest
simulation. Keep the run snapshots and image identity when comparing experiments.
The [validation record](docs/VALIDATION.md) includes a completed three-UAV camera
mission, but software rendering and solver time remain performance limits. The
Iris model has not been calibrated to the dynamics of a physical 3DR Solo.
