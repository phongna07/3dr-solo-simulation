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

## 1000 m field with five UAVs

Use these three files together:

- [big1000_5uav.yaml](../ros_ws/src/solo_mpc/scenarios/big1000_5uav.yaml): the
  supplied seed-7 geometry (60 rectangles and 48 circles), five starts at 30 m,
  33 target waypoints, and 32 leg speeds, including five 13 m/s sprints.
- [fleet_5uav.yaml](../config/fleet_5uav.yaml): `uav1` through `uav5`, independent
  SITL instances 0–4 and SYSIDs 1–5. TCP ports are 5760–5800 in increments of 10;
  JSON UDP ports are 9002–9042 in increments of 10.
- [controller_big1000.yaml](../ros_ws/src/solo_mpc/config/controller_big1000.yaml):
  30 m flight altitude, a roughly 50 m square camera footprint, 50 m LiDAR,
  and field-scale arrival and flight-operation deadlines. It leaves scenario
  speed, acceleration, horizon and map parameters intact. The default
  `controller.yaml` overrides those parameters for the small map and must not
  be paired with this scenario; a mismatched start altitude fails validation.

On the simulation machine, after building the image as described in
[Getting started](GETTING_STARTED.md), initialize the stack with:

```bash
READINESS_TIMEOUT=1200 ./scripts/dev mpc \
  --fleet /workspace/config/fleet_5uav.yaml \
  --scenario /workspace/ros_ws/src/solo_mpc/scenarios/big1000_5uav.yaml \
  --settings /workspace/ros_ws/src/solo_mpc/config/controller_big1000.yaml
```

This starts the simulation and waits for initialization with all vehicles
disarmed. Flight is a separate command: `./scripts/dev mission run`.
Stop with `./scripts/dev stop`, which requests landing and disarming first.
The existing small-map defaults remain available through `./scripts/dev mpc`.

One Gazebo server simulates the shared world. One sensor bridge, frame adapter,
target mover, MPC coordinator, independent setpoint publisher, readiness monitor,
and mission supervisor serve the whole fleet. Target filtering, leadership,
formation assignment, and the immutable neighbor-prediction snapshot are shared
in the coordinator. Each UAV has its own ArduCopter process, SITL state directory,
MAVROS process with prefixed frames, camera detector process, and persistent MPC
worker. Each MPC worker owns its map, planner, CasADi/IPOPT solver and warm start,
with four neighbor slots. Mutable native solver instances are process-local;
workers reuse the same implementation and configuration. Worker failures expire
commands through the separate publisher and supervisor; bounded shutdown cleans
up native workers. Camera recording remains optional. The GUI follow view and
optional overhead camera scale above the 30 m fleet (240 m above the target).

The Gazebo adaptation uses ground covering the field plus a 50 m border,
70 m tall obstacle prisms, and a 2 m wide magenta marker visible at 30 m. The
larger scan-height tolerance accommodates banking while discarding ground hits;
obstacle prisms extend above that scan envelope. Four cardinal follower slots
have nominal offsets of 43.75 m; the 75 m arrival radius accommodates the
formation around the final target. All vehicles spawn on the ground and take off
under their own FCU. Flight and landing have 900 wall-second deadlines for the
higher altitude on slow rendering hosts.

The port computes inflation as `ROBOT_RADIUS + INFLATE_MARGIN`, so this scenario
uses **1.5 m**, rather than the 2 m stated in the original 2D comment. Its
two-second MPC horizon reaches 20 m at VMAX. The imported 2D simulator's backup
rollout, synthetic LiDAR marching, JPS snap radii, path-deviation trigger,
`SAFETY_MARGIN`, and `TARGET_ARRIVAL_TOL` are not implemented by this ROS
controller and are omitted. This port uses measured Gazebo scans, the local
grid planner, and the mission supervisor's arrival settings instead.

Offline coverage checks five-worker numerical parity, one stationary solve at
the supplied starts with analytic obstacle scans, world/sensor generation,
launch ownership and isolation, per-leg target speeds, and worker cleanup.
No five-UAV Gazebo flight has been run for this configuration. Achieved rates,
handoffs and full-route completion still need validation on the destination
machine; this remains fixed-altitude planar MPC in a 3D physics world.

## Scenario and controller parameters

Coordinates are metres in a shared world ENU frame: east, north, up. `xlim` and
`ylim` bound the field. Rectangles are `[left, bottom, width, height]`; circles
are `[center_x, center_y, radius]`. Gazebo extrudes them into boxes/cylinders.
Target `waypoints` define a smooth route; `tar_max_speed` supplies the default
target speed. Optional `target.speeds` supplies one finite positive speed per
waypoint leg, indexed from zero. Speeds apply to arc length along each smoothed
leg, so sprints may exceed `tar_max_speed`. `TAR_SMOOTH_ENABLE: false` selects
straight waypoint segments. Optional `world_name` selects the Gazebo world and
clock/service namespace; the default is `mpc_small50`.

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
| `flight_timeout_wall` / `landing_timeout_wall` | 240 / 240 s | Flight-operation and LAND/disarm wall deadlines |

Worker timeouts bound failures; they are not the 100 ms decision target. Initial
native solves may take longer than warm solves. Increasing timeouts does not
restore sensor/control rates.
