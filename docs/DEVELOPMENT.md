# Development

[Back to the project overview](../README.md)

Use [Configuration](CONFIGURATION.md) to change an experiment and
[Operations](OPERATIONS.md) to run it. This guide describes the code boundaries
and the assumptions a contributor needs to preserve.

## Architecture

```text
scripts/dev → Docker Compose → Python ROS launch

solo_mpc/launch/mpc.launch.py
  ├─ solo_bringup/launch/simulation.launch.py
  │    └─ world generation → Gazebo + per-UAV SITL/MAVROS + readiness
  ├─ ros_gz sensor, pose, and clock bridge
  └─ frames + per-UAV marker detectors + target + controller + setpoints
       └─ controller → one persistent mapping/planning/MPC worker per UAV

solo_bringup/launch/hardware.launch.py
  └─ MAVROS + readiness (physical sensor/controller integration pending)
```

Compose manages images, containers, networks, and devices. The Bash CLI provides
short commands; ROS Python launch manages the processes inside each container.
Mission and readiness use typed ROS APIs rather than parsing CLI output. The GUI
is a separate client connected to the existing Gazebo server.

`solo_bringup` validates fleet configuration, generates worlds and runtime
configuration, creates isolated run directories, bootstraps MAVROS parameters,
and monitors readiness. `flight.py` is internal simulation takeoff/LAND support
used by the mission supervisor; it has no standalone manual flight CLI.

`solo_mpc` separates numerical planning from ROS adapters:

| Module | Responsibility |
|---|---|
| `core/mpc_problem.py`, `core/dynamics.py` | CasADi/IPOPT optimization and acceleration-lag dynamics |
| `core/planner_grid.py`, `core/planner_jps.py`, `core/geometry.py` | Grid/local planning and geometry helpers |
| `core/controller.py` | Measured-state coordination, committed paths, leader selection, follower assignment, corridors |
| `core/parallel_controller.py` | Persistent per-UAV processes, map snapshots, concurrent solves, worker deadlines and cleanup |
| `core/mapping.py` | Scan validity, occupancy updates, clearing, and teammate filtering |
| `core/tracking.py`, `core/sensors.py` | Target filtering, scan projection, camera geometry, pose history |
| `controller_node.py` | ROS inputs, references, predictions, diagnostics, and measured-state logs |
| `setpoint_node.py` | Independent publishing, command expiry, altitude feedback, proximity guard |
| `frame_node.py` | Disarmed alignment and measured state in the common world frame |
| `perception_node.py`, `target_node.py` | Camera marker detection and simulated target motion |
| `world.py`, `settings.py` | Field/sensor generation and configuration resolution |
| `mission.py`, `evaluate.py` | Simulation supervision and recorded-flight metrics |

`TeamController.step(states, scans, stamp, seers)` has no ROS imports and does not
integrate a replacement for measured vehicle state. Controller configuration and
team state are instance-local. The original solver and dynamics were retained;
simulation orchestration was replaced. Source provenance is recorded in
[UPSTREAM.json](../ros_ws/src/solo_mpc/UPSTREAM.json), with the upstream license in
[LICENSE-MPC](../ros_ws/src/solo_mpc/LICENSE-MPC). The original external
`motion_planning_MPC` checkout is not required at runtime.

The ROS controller defaults to `ParallelTeamController`, with one spawned Python
process per configured UAV. Each worker constructs only its own occupancy map,
planner, corridor and IPOPT solver, retaining its path and warm start between
cycles. All workers integrate and inflate maps concurrently. The coordinator
uses their current inflated maps for the existing fleet-wide slot checks, then
dispatches all planning/corridor/MPC jobs before collecting responses. Every
solve receives the same measured states, formation assignment and frozen neighbor
predictions, including the other two UAVs' avoidance constraints. New predictions
are committed for the next cycle after collection.

The fleet wall time includes the slowest local job in each phase, formation
assignment and IPC overhead, rather than the sum of three solves. Camera
callbacks run in separate per-UAV ROS processes. Target filtering, formation
assignment, scan projection, frame handling and setpoint publication remain
lightweight shared services. This simulates one compute owner per UAV on one
host; deploying on three Pis still needs a network transport, common frames and
synchronized timestamps. The camera algorithm remains the marker detector;
a neural-network model has not been added.

Spawn avoids inheriting ROS/DDS state or native solver objects. A shared wall
deadline bounds each worker phase. A disconnected or timed-out worker is stopped
and stays unavailable until controller restart; no late reply is reused. Missing
map data suppresses the fleet decision because formation checks need every map.
A failed solve suppresses that UAV's reference. The independent publisher still
expires references and the mission supervisor can LAND. Normal node shutdown
closes all workers, including a worker stuck in native code.

`mpc.jsonl` and `/mpc/status` report the execution mode, worker PID and cycle,
mapping, planning, corridor and solve durations, plus worker phase intervals.
These intervals can establish real overlap. Select `controller_execution:
sequential` in the settings YAML for a reference run using the same algorithm.

## Edit and rebuild

The repository is bind-mounted at `/workspace`. The entrypoint puts live project
Python modules ahead of installed copies and sources ROS, the pinned MAVROS
overlay, then the project overlay. Existing Python modules, launch files, and
configuration can be edited on the host; restart the simulation to apply them.
Rebuild the image for new executables, package metadata, or dependencies:

```bash
./scripts/dev build sim
./scripts/dev test
```

For an experimental package build inside a running container:

```bash
./scripts/dev shell
cd /workspace/ros_ws
colcon build --symlink-install
source install/setup.bash
```

That overlay affects the shell that sourced it. To use it in launched processes,
add the intended overlay sourcing to the entrypoint after validating dependencies.
Keep the order ROS → `/opt/mavros_ws` → project overlay. Persistent dependency
changes belong in the Dockerfile and locks; manual container installs disappear
when the container is replaced.

## ROS interfaces

`uavN` denotes the configured UAV name; the default fleet uses `uav1`–`uav3`.
All topic and service paths are absolute.

| Topic / service | Message type or function |
|---|---|
| `/uavN/mavros/local_position/pose`, `/uavN/mavros/local_position/velocity_local` | `PoseStamped`, `TwistStamped`: FCU measured local state |
| `/uavN/mpc/pose`, `/uavN/mpc/velocity` | `PoseStamped`, `TwistStamped`: measured common-world state |
| `/uavN/scan` | `sensor_msgs/LaserScan`: Gazebo sensor returns |
| `/uavN/camera/image_raw`, `/uavN/camera/camera_info` | `Image`, `CameraInfo`: camera images and calibration |
| `/uavN/target/observation` | `PoseStamped`: camera target observation in world coordinates |
| `/uavN/mpc/reference` | `mavros_msgs/PositionTarget`: world ENU reference |
| `/uavN/mpc/prediction` | `nav_msgs/Path`: predicted trajectory for visualization |
| `/uavN/mavros/setpoint_raw/local` | `PositionTarget`: local ENU values converted by MAVROS to NED |
| `/mpc/status`, `/mpc/setpoint_status` | `std_msgs/String`: JSON diagnostics |
| `/mpc/enable` | `std_srvs/SetBool`: enable/disable publisher ownership |
| `/target/run`, `/target/reset` | `SetBool`, `Trigger`: pause/resume and reset target |
| `/target/ground_truth`, `/uavN/ground_truth/pose` | `PoseStamped`: simulation alignment, supervision, evaluation |
| `/solo/readiness` | `std_msgs/String`: JSON fleet health |
| `/clock` | `rosgraph_msgs/Clock`: Gazebo simulation time |

## Frames and clocks

MAVROS local frames use each UAV's prefix, such as `uav1/map` and
`uav1/base_link`. MPC uses a shared `world` ENU frame. Namespaces do not establish
a transform between separate FCU origins.

The frame adapter uses simulation ground truth to calibrate translation while
disarmed, with a configured `world_from_local_yaw`. It freezes alignment, saves
`alignment.json`, and uses MAVROS measurements for subsequent motion and attitude.
The Gazebo XYZ-to-NED transform and MAVROS NED-to-ENU conversion give matching
position axes for this world. Hardware must establish a common origin and yaw
independently. Transform position with rotation plus translation; transform
velocity and acceleration with rotation only. Check the `PositionTarget`
coordinate frame, `type_mask`, and ENU/NED conversion on the target firmware.

Gazebo bridges `/clock`. MPC, perception, target, frame, and setpoint nodes use
simulation time; MAVROS uses wall/FCU time. The adapter timestamps measured state
at simulation receipt time and projects sensors against nearby measured poses.
This approximation has latency error and does not synchronize FCU measurements
with camera exposure time. Hardware requires a deliberate synchronization policy.
Setting `use_sim_time=true` without a clock source stops ROS timers. A 100 ms wall
solver duration and a 100 ms simulation period are different budgets when the
simulator runs below real time.

## Control ownership and failure behavior

The mission supervisor disables publishing during takeoff and LAND. The setpoint
publisher runs in a separate process and does not import the optimizer, so a
blocked solve does not block its publishing loop. It only controls connected,
armed UAVs in GUIDED mode with alignment and measured pose available.

Invalid or expired references cause zero horizontal velocity with altitude
feedback when state is fresh. Stale pose/FCU status causes zero velocity without
altitude feedback. Missing connection, GUIDED/armed state, pose, or alignment can
prevent output entirely. Independent command, pose, and FCU freshness limits are
listed in [Configuration](CONFIGURATION.md#timing-and-expiry).

Corridor/solver failure produces no fresh reference; failed decomposition is not
interpreted as obstacle-free space. A proximity guard stops approaching motion
near teammates, but soft optimization constraints and sampled checks do not
prove collision avoidance. Acceleration commands represent the lag model's
reference, not motor PWM or thrust. Physical deployment must identify the FCU
response and saturation behavior.

## MAVROS parameter bootstrap

The pinned MAVROS commit uses `use_global_arguments(false)` on plugin nodes,
so a generated command-line YAML file alone does not configure them. Bootstrap
uses `ListParameters`, `GetParameters`, and `SetParameters`, compares existing
values, and writes only differences. Readiness requires successful completion.
Undeclared upstream keys are skipped with warnings; a rejected declared parameter
is an initialization error. UAS frames are configured separately on `mavros_node`.

The pinned `setpoint_attitude.use_quaternion` callback captures a constructor-local
QoS object by reference. Calling it later can cause an `invalid allocator` crash.
Bootstrap avoids writing the already-correct default `false` and rejects changing
that mode. Attitude/quaternion control requires fixing the callback to retain QoS
by value, rebuilding MAVROS, and validating a new environment release. Current
position/velocity control does not require that mode change.

## Verification

Run `./scripts/dev test` for configuration, world generation, controller,
sensor/target logic, ROS construction and transforms, stale-input behavior, mission
guards, and CLI shutdown ordering. Tests run in the simulation image with the
project bind-mounted. Use `git diff --check` for whitespace review.

The [simulation workflow](../.github/workflows/simulation.yml) builds the image,
runs tests, and starts the three-UAV stack to check heartbeat and pose. It does
not arm or fly a mission. Changes to flight behavior need a supervised simulation
experiment and review of measured logs; update the [validation record](VALIDATION.md)
with the configuration, image identity, evidence, and remaining limits. Hardware
verification follows the staged [migration plan](HARDWARE.md).
