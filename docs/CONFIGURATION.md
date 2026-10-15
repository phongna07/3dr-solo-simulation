# Configuration

[Back to the project overview](../README.md)

Stop the current simulation before changing configuration, then start a fresh
run. The mission snapshots its inputs at launch; editing YAML does not update an
active experiment. MPC runs cannot resume old alignment or sensor state.

## Choose the inputs

| Input | Default file | Purpose |
|---|---|---|
| Fleet | [config/fleet.yaml](../config/fleet.yaml) | UAV names, SITL instances, MAVLink system IDs |
| Scenario | [small50_3uav.yaml](../ros_ws/src/solo_mpc/scenarios/small50_3uav.yaml) | Field bounds, starts, obstacles, target route, scenario parameters |
| Controller | [controller.yaml](../ros_ws/src/solo_mpc/config/controller.yaml) | Flight altitude, sensors, timing, frame calibration, MPC overrides |
| Hardware | [config/hardware.yaml](../config/hardware.yaml) | Physical FCU links and sensor integration placeholders |

Select custom simulation files with container paths. The repository root is
mounted at `/workspace`:

```bash
./scripts/dev mpc \
  --fleet /workspace/config/fleet-custom.yaml \
  --scenario /workspace/ros_ws/src/solo_mpc/scenarios/custom.yaml \
  --settings /workspace/ros_ws/src/solo_mpc/config/controller.yaml
```

For persistent selection, edit `.env`:

| Variable | Default / meaning |
|---|---|
| `FLEET_CONFIG` | `/workspace/config/fleet.yaml` |
| `MPC_SCENARIO` | `/workspace/ros_ws/src/solo_mpc/scenarios/small50_3uav.yaml` |
| `MPC_SETTINGS` | `/workspace/ros_ws/src/solo_mpc/config/controller.yaml` |
| `MPC_TARGET_MODE` | Empty uses the controller file; override with `camera` or `ground_truth` |
| `READINESS_TIMEOUT` | 600 wall seconds for initial bringup |
| `SIM_IMAGE`, `COMPANION_IMAGE` | Image tags or registry digests |
| `BUILD_JOBS` | 4 compiler jobs for the simulation build |
| `ROS_DOMAIN_ID` | 42; discovery stays inside the simulation container |
| `COMPOSE_PROJECT_NAME` | `solo`; also selects the Gazebo partition |
| `LIBGL_ALWAYS_SOFTWARE` | Empty by default; `1` requests software rendering |

`scripts/dev` supplies the host UID/GID and a new run ID automatically. Use
[WSL and GPU](WSL_GPU.md) for `COMPOSE_FILE` overrides and
[Hardware migration](HARDWARE.md) for hardware-specific device variables.

## Change fleet size

Fleet size comes from the configuration. For N UAVs:

1. Provide N fleet entries with unique `name`, `instance`, and `sysid` values.
2. Provide N finite XYZ rows in `scenario.starts`, in fleet order. Start XY
   positions must be at least `UAV_SAFE_DISTANCE` apart.
3. Provide at least N − 1 unique, nonzero XY formation offsets in
   `params.GRID_CELLS`. One UAV is the leader; the others need follower slots.

The schema requires `schema_version: 1`, a nonempty `uavs` list, and a valid
`world_name`. Names match `[a-z][a-z0-9_]*`, system IDs are integers from 1 to 255,
and SITL instances are integers from 0 to 200. Every `spawn` has six finite values:
`[x, y, z, roll, pitch, yaw]`, with angles in degrees. MPC launch replaces these
with scenario XY, ground height 0.195 m, and orientation `[0, 0, 90]`; flight
altitude comes from the controller settings.

For four UAVs, append this entry to the fleet:

```yaml
- name: uav4
  instance: 3
  sysid: 4
  spawn: [0.6, 1.8, 0.195, 0, 0, 90]
```

Append `[0.6, 1.8, 3]` to the default scenario's `starts`, and set its
`params.GRID_CELLS` to `[[0, 1], [0, -1], [-1, 0]]`. Check whether your controller
file overrides these values. Count mismatches and insufficient slots are rejected
before launch; each new fleet size still needs its own mission validation.

SITL instance `i` uses MAVLink TCP port `5760 + 10*i` and Gazebo JSON UDP port
`9002 + 10*i`. These ports stay inside the container; see
[Operations](OPERATIONS.md#topology-and-readiness) for the default topology.

## Scenario and MPC parameters

Scenario coordinates use metres in the common world ENU frame. `xlim` and `ylim`
bound the field. Rectangles are `[left, bottom, width, height]`; circles are
`[center_x, center_y, radius]`. Gazebo extrudes these footprints into solid boxes
and cylinders. Target `waypoints` define a smooth route, and `tar_max_speed`
controls its speed.

Algorithm parameters resolve in this order:

```text
settings.py defaults → scenario.params → controller.params
```

The loader then derives `TIMESTEP` from `control_period`, `VIEWING_RADIUS` from
scenario `viewing_radius`, world bounds from the field limits, obstacle inflation
from radius plus margin, and standoff distance from viewing radius and target
speed. These derived values take precedence over `params` entries. The resolved
configuration is saved in the run's `mpc-config.json`.

| Setting / parameter | Current default | Meaning |
|---|---:|---|
| `flight_altitude` | 3 m | Fixed flight altitude |
| `obstacle_height` | 5 m | Must exceed altitude plus vehicle radius |
| `control_period` | 0.1 s | Desired MPC period in simulation time |
| `publish_rate` | 20 Hz | Independent setpoint publisher in simulation time |
| `ROBOT_RADIUS` / `INFLATE_MARGIN` | 0.36 / 0.15 m | Vehicle footprint and obstacle margin |
| `UAV_SAFE_DISTANCE` | 1 m | Minimum initial spacing and separation parameter |
| `VMAX` / `UMAX` | 1.2 m/s / 2 m/s² | Horizontal speed / acceleration limits |
| `ACCEL_TAU` | 0.2 s | Acceleration-lag model time constant |
| `CORRIDOR_MAX_FACES` | 16 | Initial corridor allocation; grows when needed |
| `MPC_SOLVER` | `ipopt` | Supported solver backend |

`IPOPT_OPTIONS` controls solver iterations, tolerances, and CPU budget. The default
per-solve CPU limit is 0.06 s, which does not guarantee a whole-fleet cycle fits
the 0.1 s control period. Identify actual vehicle response before increasing
motion limits. The planner is planar, and separation/visibility constraints are
soft; these parameters do not establish collision guarantees.

## Sensors and target modes

The default camera is 160 × 160 at 8 Hz with a horizontal field of view of about
53.13°. At 3 m altitude it sees approximately a 3 m square ground footprint. The
horizontal LiDAR has 180 samples at 10 Hz. Sensor rates follow simulation time.
No gimbal or video encoder is used in the mission model.

`camera` mode uses image detections to update the target estimate. The controller
does not subscribe to target ground truth in this mode. Ground truth still
supports initial disarmed frame alignment, mission route-completion checks, and
evaluation. LiDAR always uses Gazebo sensor returns.

For a controller experiment using target ground truth, start a fresh run with:

```bash
MPC_TARGET_MODE=ground_truth ./scripts/dev mpc --seconds 20
```

Missing scan rays remain unknown; observed no-return rays clear space. Strongly
tilted rays are discarded. The default `UNKNOWN_POLICY` is `optimistic`, so
unobserved space is not guaranteed clear. A body-mounted 2D scan cannot describe
arbitrary 3D obstacles. The camera detector assumes a known flat target plane.

## Timing and expiry

| Setting | Default | Purpose |
|---|---:|---|
| `controller_execution` | `parallel` | One persistent local compute worker per UAV; `sequential` for comparison |
| `controller_worker_timeout_wall` | 1 s | Shared deadline per mapping/solve phase; timeout stops the worker |
| `controller_worker_startup_timeout_wall` | 30 s | Shared worker initialization deadline |
| `state_timeout_sim`, `sensor_timeout_sim` | 0.5 s each | Controller input freshness |
| `target_timeout_sim` | 1.5 s | Observation age before target-loss handling |
| `target_search_timeout_sim` | 8 s | Supervisor's reacquisition deadline |
| `planner_failure_timeout_sim` | 3 s | Supervisor's persistent-failure deadline |
| `command_timeout_sim` / `command_timeout_wall` | 0.4 / 3 s | Independent command expiry limits |
| `pose_timeout_wall` | 3 s | Measured pose freshness |
| `fcu_timeout_sim` / `fcu_timeout_wall` | 2.5 / 60 s | Separate freshness limits for FCU heartbeats, which follow simulation time |
| `arrival_fleet_radius` / `arrival_timeout_sim` | 6 m / 30 s | Fleet arrival after the target reaches its last waypoint |

`*_sim` values use Gazebo time; `*_wall` values use elapsed real time. At a
real-time factor below 1, these are different budgets. Increasing timeouts does
not restore sensor or control rates. See [Development](DEVELOPMENT.md#frames-and-clocks)
for frame alignment and timestamp handling.

Worker timeouts bound failures; they are not the 100 ms decision target. The first
native solve can be slower than later warm solves. See [Development](DEVELOPMENT.md#architecture)
for worker ownership, synchronization and shutdown behavior.
