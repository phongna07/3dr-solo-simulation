# Configuration

[Overview](../README.md) · [Simulation](SIMULATION.md) · [Development](DEVELOPMENT.md)

Stop the simulation before changing inputs. Launch snapshots configuration;
editing a YAML file does not reconfigure an active experiment.

## Input files

| Input | Default | Purpose |
|---|---|---|
| Fleet | [config/fleet.yaml](../config/fleet.yaml) | UAV names, SITL instances, SYSIDs |
| Scenario | [small50_3uav.yaml](../ros_ws/src/solo_mpc/scenarios/small50_3uav.yaml) | Bounds, starts, obstacles, target route, algorithm parameters |
| Controller | [controller.yaml](../ros_ws/src/solo_mpc/config/controller.yaml) | Altitude, sensors, execution, timing, parameter overrides |

Use container paths for CLI overrides:

```bash
./scripts/dev mpc --fleet /workspace/config/fleet-custom.yaml   --scenario /workspace/config/scenario-custom.yaml   --settings /workspace/config/controller-custom.yaml
```

`.env` supports `FLEET_CONFIG`, `MPC_SCENARIO`, `MPC_SETTINGS`, `MPC_TARGET_MODE`,
`READINESS_TIMEOUT` (600 wall seconds), `SIM_IMAGE`,
`ROS_DOMAIN_ID` (42), `BUILD_JOBS` (4), and `LIBGL_ALWAYS_SOFTWARE`.
The runner supplies host UID/GID and a fresh run ID. Select GPU/display overrides
as described in [Simulation](SIMULATION.md#gpu-and-display).

### Five UAVs in a 200 × 200 m map

`./scripts/dev mpc --preset large200_5uav --record-video` selects these matching inputs:

- [config/fleet-5uav.yaml](../config/fleet-5uav.yaml): five independent SITL/MAVROS instances.
- [large200_5uav.yaml](../ros_ws/src/solo_mpc/scenarios/large200_5uav.yaml): 200 × 200 m bounds, five starts, four cross-shaped formation slots, 240 random rectangles, 240 random circles, and a smooth route with early slalom bends.
- [controller-large200.yaml](../ros_ws/src/solo_mpc/config/controller-large200.yaml): 3 m altitude, 2.2 m/s MPC speed bound for all UAVs, 6 m LiDAR range and a 0.35 s solver CPU cap. Target speed is 1.0 m/s. Actual UAV speed varies through turns and formation contraction and can briefly overshoot the prediction bound. Worker deadlines, expiry and supervision remain enabled.

The leader starts at `[16,16,3]` with the target at `[16,16]`, centered in its
downward camera footprint. Followers start 2.625 m left, right, behind and ahead.
The large map uses a 0.3 m grid to bound per-worker mapping cost. Obstacles have
1.5–4 m rectangle sides or 0.6–1.5 m circle radii and retain 5 m collision height.
The generator uses the same smoothed route as the target and protects every
sampled route segment with at least 2.0 m clearance from enclosing obstacle discs.
It attempts nearby obstacles on both sides every 4 m of route distance, with
2.1–3.0 m disc-edge offsets, then fills the remaining field randomly. Enclosing
discs retain at least 0.5 m between obstacles; these side gaps can be too narrow
for a UAV and are not designated passages. Seed `200205` has 162 nearby
obstacles among 480 total, with 2.114 m minimum sampled route-to-footprint
clearance. Starts retain at least 3 m clearance from obstacle discs. Compared
with the previous realization, the route band shrinks from 3.2 m per side and
roadside placement spacing from 5.5 m, while target/UAV speed limits increase
from 0.6/1.5 m/s. Acceleration remains limited to 2 m/s².
The controller still learns obstacles through LiDAR and receives no future route;
followers may contract formation as they approach obstacles.
This preset uses `overhead_camera_offset: [-12,-12,40]` for a higher angled
recording/GUI view: 12 m west, 12 m south and 40 m above the target. The wider
view shows the five-UAV fleet and surrounding buildings/trees; occasional
occlusion is expected. It affects the observer camera only; UAVs still fly at
3 m with their existing cameras and LiDAR.

The checked-in realization uses seed `200205`. It repeats on each launch so
failures can be reproduced. Generate another realization before starting:

```bash
./scripts/generate-large-scenario --seed 42 --output config/large200-seed42.yaml
./scripts/dev mpc --preset large200_5uav --scenario /workspace/config/large200-seed42.yaml --record-video
```

The generator needs Python 3 with NumPy and imports this project's target spline.
Alternatively, run it before starting the stack with
`docker compose run --rm --no-deps sim python3 /workspace/scripts/generate-large-scenario`.
Omitting `--output` replaces the supplied large scenario. Keep generated
experiments in `artifacts/` when they should not be committed. Different seeds
need their own simulator verification. Explicit
`--fleet`, `--scenario`, and `--settings` arguments after `--preset` override that
preset; the original no-argument defaults remain unchanged.

## Fleet and geometry

The supplied three-UAV scenario starts at `[5,2,3]`, `[2.5,2,3]`, `[7.5,2,3]`
in fleet order, facing north (+Y). The target starts at `[5,3]` and follows a
smooth winding route to `[47,23]` at **0.6 m/s**. Horizontal UAV speed is
**1.2 m/s** under the supplied controller overrides.

Coordinates are metres in common world ENU: east, north, up. `xlim`/`ylim`
bound the field; rectangles are `[left,bottom,width,height]`, circles
`[center_x,center_y,radius]`. Obstacles are extruded to `obstacle_height`.
Decorative towers/trees can extend above their collision/LiDAR primitives;
planning uses the original footprints. Coverage guides are render-only.

Match fleet entries to finite XYZ `starts` rows and supply one unique, nonzero
`params.GRID_CELLS` offset per follower. Start altitude must match
`flight_altitude`; XY starts must meet `UAV_SAFE_DISTANCE`. Fleet schema is `1`,
names match `[a-z][a-z0-9_]*`, SYSIDs are 1–255, and instances are 0–200.
Instance `i` uses MAVLink TCP `5760+10*i` and JSON UDP `9002+10*i`, inside the
container. MPC launch replaces fleet spawn positions with scenario XY and a
0.195 m ground height; flight altitude is controlled separately.

Target `waypoints` define the route. Optional `target.speeds` supplies a positive
speed per leg. `TAR_SMOOTH_ENABLE: false` selects a polyline; otherwise a
centripetal spline uses `TAR_SPLINE_DS`. The controller predicts from observed
position/estimated velocity; it does not read the future route.

## Algorithm parameters

Resolution order is local `settings.py` defaults → scenario `params` → controller
`params` → derived values. Upstream `config.py` is never imported. Inspect the
run's resolved `config/mpc-config.json` rather than inferring values from a YAML
file that may have been overridden.

| Parameter | Supplied value | Meaning |
|---|---:|---|
| `HORIZON_LENGTH` / `TIMESTEP` | 10 / 0.1 s | Nominal prediction horizon; timestep follows `control_period` |
| `ROBOT_RADIUS` / `INFLATE_MARGIN` | 0.36 / 0.15 m | Body and grid safety margin |
| `VMAX` / `UMAX` / `ACCEL_TAU` | 1.2 m/s / 2 m/s² / 0.2 s | Prediction limits and acceleration lag |
| `UAV_SAFE_DISTANCE` / `COLLISION_AVOID_DISTANCE` | 1 / 1.5 m | Soft separation and avoidance cost distance |
| `GRID_RESOLUTION` / `LOCAL_PLAN_RADIUS` | 0.15 / 10 m | Grid resolution and local window half size |
| `LOCAL_SEARCH` | `jps` | Local JPS with Dijkstra fallback; `dijkstra` applies unknown-space cost |
| `UNKNOWN_POLICY` / `UNKNOWN_COST` | `optimistic` / 1.5 | Unknown cells allowed, with higher Dijkstra cost |
| `CONTINGENCY` / `BACKUP_HORIZON` | true / 25 | Separate stopping branch ending at rest after 2.5 s |
| `W_backup` / `TERMINAL_STOP` | 0.1 / false | Backup effort weight / optional nominal terminal stop |
| `CORRIDOR_BOX` / `CORRIDOR_MAX_FACES` | 1.5 m / 16 | Decomposition half size and initial solver allocation |
| `SPEED_RECOVERY` | true | Local adapter recovery for initially overspeed measured states |
| `IPOPT_OPTIONS` | 500 iterations / 0.2 s CPU | Solver limits; worker deadline remains independent |

The supplied controller enables `SPEED_RECOVERY` in the local adapter. It permits
only the initial predicted overspeed that its maximum-effort braking model cannot
remove immediately, then restores the nominal speed bound. Measured state,
corridor and collision constraints, solver CPU cap, and command expiry remain
in effect; the upstream source is unchanged. Omit the override or set it to
`false` to use the adapter's default strict speed bounds.

Derived values include inflation radius, world bounds, standoff distance,
formation thresholds, and corridor margin. Unless overridden, `CORRIDOR_MARGIN`
includes body radius, three Gaussian LiDAR-noise standard deviations, ray spacing,
and the grid-cell half diagonal. Noise is unbounded. The forward corridor seed
must leave at least one cell of usable range. Keep obstacle collision height
above flight altitude plus body radius.

After a nominal solve fails, a retained stop branch can be replayed only while
measured XY position, velocity, and acceleration agree within 0.15 m, 0.3 m/s,
and 0.5 m/s². Otherwise no fresh reference is produced; independent expiry and
supervision apply. Failed nominal solves remain visible as `nominal_ok: false`.
These sampled/soft constraints do not establish collision guarantees.

## Sensors and target input

Camera defaults are 160 × 160 at 8 Hz, square images, and approximately 53.13°
horizontal FOV. The ideal Gazebo gimbal keeps the camera downward and world-aligned,
with its optical center 0.04 m below the body. It cancels attitude motion without
mechanical dynamics. The ground footprint is about 2.96 × 2.96 m at 3 m body
altitude. Projection assumes a known flat target plane; occlusion remains possible.

Horizontal body-mounted LiDAR uses 360 rays at 10 Hz, 3 m range, and 0.01 m
Gaussian range-noise standard deviation. Invalid/missing rays remain unknown;
observed no-return rays clear space; excessive tilt is rejected. An optimistic
map is not proof that unobserved space is clear.

`camera` mode uses magenta-marker detections. During brief loss the estimate
coasts, then searches; prolonged loss pauses the cooperative simulated target.
`./scripts/dev mpc --ground-truth-target` instead provides exact timestamped target
positions, omitting detectors/cameras unless recording. Velocity, confidence,
and lead pursuit remain estimated. A stale external target fails the mission
with landing cleanup. The CLI flag overrides environment/YAML target selection.

`SEARCH` means target pursuit; a distant UAV can be in SEARCH even with fresh
external input. Formation membership still uses viewing-distance gates. The
resolved source and `target_input_fresh` are reported in logs and status.

## Timing and expiry

`*_sim` uses Gazebo time; `*_wall` uses elapsed real time. Slow simulation makes
these budgets differ. Sensor rates also follow simulation time.

| Setting | Default |
|---|---:|
| `flight_altitude` / `obstacle_height` | 3 / 5 m |
| `control_period` / `publish_rate` | 0.1 s / 20 Hz |
| `controller_execution` | `parallel`; `sequential` orders the same persistent workers |
| Worker phase / startup timeout | 1 / 30 wall seconds |
| State / sensor freshness | 0.5 / 0.5 simulation seconds |
| Target freshness / search budget | 1.5 / 8 simulation seconds |
| Persistent planning failure budget | 3 simulation seconds |
| Command expiry | 0.4 simulation seconds and 3 wall seconds |
| Pose wall freshness | 3 seconds |
| FCU freshness | 2.5 simulation seconds and 60 wall seconds |
| Final fleet arrival radius / timeout | 6 m / 30 simulation seconds |
| `diagnostic_capture_cycles` | 128 retained calls per UAV; 0 disables capture |

Timeouts bound failures; they do not guarantee a 100 ms fleet cycle. Native
planner kernels warm both contiguous and sliced windows before worker readiness.
See [Development](DEVELOPMENT.md#frames-and-clocks) before changing clock policy.
