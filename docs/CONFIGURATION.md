# Configuration

[Project overview](../README.md) · [Operations](OPERATIONS.md)

Stop the simulation before editing inputs, then launch a fresh run. Inputs are
snapshotted at launch; editing YAML does not change an active experiment.

## Select input files

| Input | Default file | Purpose |
|---|---|---|
| Fleet | [fleet.yaml](../config/fleet.yaml) | UAV names, SITL instances, MAVLink system IDs |
| Scenario | [small50_3uav.yaml](../ros_ws/src/solo_mpc/scenarios/small50_3uav.yaml) | Field, starts, obstacles, target route, algorithm parameters |
| Controller | [controller.yaml](../ros_ws/src/solo_mpc/config/controller.yaml) | Altitude, sensors, timing, frame calibration, parameter overrides |
| Hardware | [hardware.yaml](../config/hardware.yaml) | Physical FCU links and sensor placeholders |

Use container paths for CLI overrides. The checkout is mounted at `/workspace`:

```bash
./scripts/dev mpc \
  --fleet /workspace/config/fleet-custom.yaml \
  --scenario /workspace/config/scenario-custom.yaml \
  --settings /workspace/config/controller-custom.yaml
```

For persistent selection, edit `.env`:

| Variable | Default or meaning |
|---|---|
| `FLEET_CONFIG` | `/workspace/config/fleet.yaml` |
| `MPC_SCENARIO` | `/workspace/ros_ws/src/solo_mpc/scenarios/small50_3uav.yaml` |
| `MPC_SETTINGS` | `/workspace/ros_ws/src/solo_mpc/config/controller.yaml` |
| `MPC_TARGET_MODE` | Empty uses controller settings; override with `camera` or `ground_truth` |
| `READINESS_TIMEOUT` | 600 wall seconds for initial simulation readiness |
| `SIM_IMAGE`, `COMPANION_IMAGE` | Development tags or registry digests |
| `BUILD_JOBS` | 4 compiler jobs by default |
| `ROS_DOMAIN_ID` | 42; simulation discovery is local to the container |
| `COMPOSE_PROJECT_NAME` | `solo`; also selects the Gazebo partition |
| `LIBGL_ALWAYS_SOFTWARE` | Empty by default; `1` requests software rendering |

The CLI supplies host UID/GID and a fresh run ID. See [WSL and GPU setup](WSL_GPU.md)
for Compose overrides and [Hardware migration](HARDWARE.md) for device variables.
Release deployments use a [separate environment file](REPRODUCIBILITY.md).

## Horizontal launch row, scattered obstacle field

The default [small50_3uav.yaml](../ros_ws/src/solo_mpc/scenarios/small50_3uav.yaml)
uses a 50 m east-west by 30 m north-south field with twelve rectangular and
twelve circular obstacles at irregular positions with varied sizes. The three
UAVs start at `[5, 2, 3]`,
`[2.5, 2, 3]`, and `[7.5, 2, 3]` in fleet order. They share Y = 2 m and altitude
3 m, forming a horizontal row with 2.5 m spacing. All face north (+Y, yaw 90°),
perpendicular to the row. The target begins at `[5, 3]`, ahead of the middle
UAV, initially heading north. Its smooth route sweeps across the field, with
east/west turns and changes between northward and southward travel, finishing
at `[47, 23]`.

Start a fresh simulation with the existing three-UAV fleet and controller:

```bash
./scripts/dev mpc
```

MPC launch takes the UAV positions from the scenario, so no fleet file
changes are needed. The row defines the initial formation; the controller can
adjust UAV positions while following the target and avoiding obstacles.

## 1000 m field with five UAVs

The larger experiment uses these three files together:

| Input | File |
|---|---|
| Scenario | [big1000_5uav.yaml](../ros_ws/src/solo_mpc/scenarios/big1000_5uav.yaml) |
| Fleet | [fleet_5uav.yaml](../config/fleet_5uav.yaml) |
| Controller | [controller_big1000.yaml](../ros_ws/src/solo_mpc/config/controller_big1000.yaml) |

On the simulation machine, start it with:

```bash
./scripts/dev mpc \
  --fleet /workspace/config/fleet_5uav.yaml \
  --scenario /workspace/ros_ws/src/solo_mpc/scenarios/big1000_5uav.yaml \
  --settings /workspace/ros_ws/src/solo_mpc/config/controller_big1000.yaml
./scripts/dev mission run
```

The first command starts the simulation and waits for initialization; the
second arms the fleet and flies until route completion. Default three-UAV
inputs remain selected when no overrides are supplied. Use all three larger-map
files: the default fleet has only three entries, and the default controller
would override the larger scenario's speeds, range, inflation, and corridor size.

This scenario preserves the supplied five starts, 27 waypoints, 26 leg speeds,
33 rectangles, and 32 circles from the frozen seed-7 map. Flight altitude is
30 m, obstacles extend to 40 m, and the downward camera sees approximately a
50 m square at flight height. A 2 m square magenta marker remains detectable
in the 160 × 160 image. UAV 3 starts within the initial target footprint;
the other UAVs join from their supplied positions. Four cardinal follower
slots have nominal spacing 43.75 m. The larger arrival radius accommodates
this formation. The ground surface covers the whole map with a 50 m border.

`target.speeds` specifies one positive speed per waypoint leg. Speeds apply to
arc length along the sampled spline, switching at the original waypoints.
Zero-based legs 5, 7, 10, and 17 use 13 m/s; all others use 5 m/s. Without
`target.speeds`, every leg uses `tar_max_speed`. Sprints exceed the UAV's
10 m/s limit and exercise target reacquisition and leadership changes; hand-offs
depend on measured camera visibility and vehicle motion.

The port uses Gazebo LiDAR returns rather than 2D ray marching, so
`LIDAR_MARCH_STEP` is omitted. The planner is always the local planner;
the upstream `PLANNER` selector and JPS-only `GOAL_SNAP_RADIUS` are omitted.
Upstream `PATH_DEVIATION_THRESHOLD`, `SAFETY_MARGIN`, and `TARGET_ARRIVAL_TOL`
are also omitted because this port uses committed-path clearance checks,
vehicle/obstacle margins, and mission arrival checks respectively. Target
observation noise, minimum LiDAR range/noise, and start snapping are carried
through to their corresponding ROS/controller paths. Total obstacle inflation
is 1.5 m (`ROBOT_RADIUS: 0.5` plus `INFLATE_MARGIN: 1.0`). The 0.1 s timestep
and vehicle radius do not establish a no-tunnelling guarantee at 10 m/s.

These are starting flight parameters. Automated checks cover configuration,
generated SDF/bridges, target speed timing, and five numerical MPC workers;
the larger experiment still needs flight validation and timing measurements
on the simulation machine.

## Change fleet size

For N UAVs, provide:

1. N fleet entries with unique `name`, `instance`, and `sysid` values.
2. N finite XYZ rows in `scenario.starts`, in fleet order, with XY spacing at
   least `UAV_SAFE_DISTANCE`.
3. At least N − 1 unique, nonzero XY offsets in `params.GRID_CELLS` for followers.

Fleet schema version is `1`. Names match `[a-z][a-z0-9_]*`, system IDs are 1–255,
and SITL instances are 0–200. The fleet needs a valid `world_name`. Each `spawn`
contains `[x, y, z, roll, pitch, yaw]`, with finite values and angles in degrees.
MPC launch uses scenario XY starts, ground height 0.195 m, and orientation
`[0, 0, 90]`; `flight_altitude` controls flight height.

For a fourth UAV, append this entry to the fleet's `uavs` list:

```yaml
- name: uav4
  instance: 3
  sysid: 4
  spawn: [0.6, 1.8, 0.195, 0, 0, 90]
```

Append `[0.6, 1.8, 3]` to the default scenario's `starts`, and set
`params.GRID_CELLS` to `[[0, 1], [0, -1], [-1, 0]]`. Check controller overrides.
Count/slot mismatches fail before launch; validate each new fleet size in flight.
Instance `i` uses MAVLink TCP `5760 + 10*i` and JSON UDP `9002 + 10*i` internally.

## Scenario and controller parameters

Coordinates are metres in a shared world ENU frame: east, north, up. `xlim` and
`ylim` bound the field. Rectangles are `[left, bottom, width, height]`; circles
are `[center_x, center_y, radius]`. Gazebo extrudes them into boxes/cylinders.
Target `waypoints` define a smooth route; `tar_max_speed` supplies the default
target speed, and optional `target.speeds` overrides it per waypoint leg.

Algorithm parameters resolve in this order:

```text
settings.py defaults → scenario.params → controller.params → derived values
```

Derived values include timestep from `control_period`, viewing radius from the
scenario, world bounds, inflation from vehicle radius plus margin, and standoff
distance from viewing radius and target speed. Inspect the final `mpc-config.json`
in the run directory rather than relying on an overridden YAML value.

| Setting / parameter | Default | Meaning |
|---|---:|---|
| `flight_altitude` / `obstacle_height` | 3 / 5 m | Flight height / extruded obstacle height |
| `control_period` / `publish_rate` | 0.1 s / 20 Hz | MPC decisions / independent setpoint publishing, in simulation time |
| `HORIZON_LENGTH` | 10 steps | MPC prediction horizon |
| `ROBOT_RADIUS` / `INFLATE_MARGIN` | 0.36 / 0.15 m | Vehicle footprint / obstacle margin |
| `UAV_SAFE_DISTANCE` | 1 m | Initial spacing and separation parameter |
| `VMAX` / `UMAX` | 1.2 m/s / 2 m/s² | Horizontal speed / acceleration limits |
| `ACCEL_TAU` | 0.2 s | Acceleration-lag time constant |
| `CORRIDOR_MAX_FACES` | 16 | Initial solver allocation; grows if needed |
| `MPC_SOLVER` | `ipopt` | Solver backend |

Keep obstacle height above flight altitude plus vehicle radius. `IPOPT_OPTIONS`
sets iterations, tolerances, and CPU limits; the default 0.06 s per-solve limit
does not guarantee a 0.1 s fleet cycle. Planning is planar and separation/visibility
constraints are soft, so these settings do not establish collision guarantees.

## Sensors and target input

Defaults are a 160 × 160 camera at 8 Hz, approximately 53.13° horizontal FOV,
and horizontal LiDAR with 180 samples at 10 Hz. At 3 m the camera sees roughly
a 3 m square of ground. Rates follow simulation time. The mission model has no
gimbal; the optional recorders encode video separately.

`camera` mode uses marker detections for target tracking; the controller does
not subscribe to target truth. Ground truth still supports disarmed frame
alignment, route-completion checks, and evaluation. To use truth as controller
input instead, start a fresh run:

```bash
MPC_TARGET_MODE=ground_truth ./scripts/dev mpc
./scripts/dev mission run --seconds 20
```

LiDAR always uses Gazebo returns. Missing rays stay unknown; observed no-return
rays clear space; strongly tilted rays are discarded. Default `UNKNOWN_POLICY`
is `optimistic`, so unobserved space is not guaranteed clear. A 2D scan cannot
represent arbitrary 3D obstacles, and camera projection assumes a known flat
target plane.

`scan_projection_mode` defaults to `horizontal`: endpoints must remain within
`scan_height_tolerance` (1 m) of the sensor height, and above 0.3 m. The
controller requires at least 70% of the scan's rays to survive projection.
At a 3 m range, a 5° pitch preserves the full scan. At a 50 m range, that same
pitch preserves only about 14%; even 2° preserves only about 39%. This can
produce `insufficient_valid_scan` immediately after horizontal acceleration,
then trigger the mission's planner-failure timeout and LAND.

The five-UAV controller selects `scan_projection_mode: extruded` for its known
ground-to-40 m boxes and cylinders. This mode relies on obstacle XY footprints
being constant throughout the projection band. It clips each measured ray to
the band from 0.3 m to `obstacle_height - ROBOT_RADIUS` (39.5 m for this scenario),
then projects the retained prefix into XY. Returns inside the band remain
obstacle hits; clipped endpoints only clear the observed prefix. Space beyond
the band and missing/invalid rays stay unknown. If the sensor itself is outside
the band, the scan is rejected. This mode does not use `scan_height_tolerance`
and is unsuitable for arbitrary 3D geometry such as overhangs or varied building
heights. The 70% validity requirement remains active in both modes.

`mpc.jsonl` and `/mpc/status` include `scan_diagnostics` for processed vehicles:
projection mode, total/retained rays, retained fraction, range, sensor height,
and measured roll/pitch. Use these fields to identify which vehicle causes a
scan rejection before adjusting projection or timeout settings.

## Timing and expiry

`*_sim` means Gazebo time; `*_wall` means elapsed real time. These budgets differ
when simulation runs below real time. See [frames and clocks](DEVELOPMENT.md#frames-and-clocks).

| Setting | Default | Purpose |
|---|---:|---|
| `controller_execution` | `parallel` | Persistent worker per UAV; `sequential` for comparison |
| `controller_worker_timeout_wall` / `controller_worker_startup_timeout_wall` | 1 / 30 s | Shared phase / initialization deadlines |
| `state_timeout_sim` / `sensor_timeout_sim` | 0.5 / 0.5 s | Input freshness |
| `target_timeout_sim` / `target_search_timeout_sim` | 1.5 / 8 s | Target loss / reacquisition deadline |
| `planner_failure_timeout_sim` | 3 s | Persistent planning-failure deadline |
| `command_timeout_sim` / `command_timeout_wall` | 0.4 / 3 s | Independent reference expiry limits |
| `pose_timeout_wall` | 3 s | Measured pose freshness |
| `fcu_timeout_sim` / `fcu_timeout_wall` | 2.5 / 60 s | FCU heartbeat freshness |
| `arrival_fleet_radius` / `arrival_timeout_sim` | 6 m / 30 s | Fleet arrival after the last target waypoint |

Worker timeouts bound failures; they are not the 100 ms decision target. Initial
native solves may take longer than warm solves. Increasing timeouts does not
restore sensor/control rates.
