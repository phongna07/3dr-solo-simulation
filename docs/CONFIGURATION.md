# Configuration

[Overview](../README.md) · [Simulation](SIMULATION.md) · [Development](DEVELOPMENT.md)

Stop the simulation before changing inputs. Launch snapshots configuration;
editing a YAML file does not reconfigure an active experiment.

## Input files

| Input | Default | Purpose |
|---|---|---|
| Fleet | [config/fleet.yaml](../config/fleet.yaml) | UAV names and native Gazebo spawn poses |
| Scenario | [small50_3uav.yaml](../ros_ws/src/solo_mpc/scenarios/small50_3uav.yaml) | Bounds, starts, obstacles, target route, algorithm parameters |
| Controller | [controller.yaml](../ros_ws/src/solo_mpc/config/controller.yaml) | Altitude, sensors, execution, timing, parameter overrides |

Use container paths for CLI overrides:

```bash
./scripts/dev mpc --fleet /workspace/config/fleet-custom.yaml   --scenario /workspace/config/scenario-custom.yaml   --settings /workspace/config/controller-custom.yaml
```

`.env` supports `FLEET_CONFIG`, `MPC_SCENARIO`, `MPC_SETTINGS`, `MPC_TARGET_MODE`,
`SIM_IMAGE`,
`ROS_DOMAIN_ID` (42), `BUILD_JOBS` (4), and `LIBGL_ALWAYS_SOFTWARE`.
The runner supplies host UID/GID and a fresh run ID. Select GPU/display overrides
as described in [Simulation](SIMULATION.md#gpu-and-display).

Readiness waits indefinitely, including on slow hosts where native model/sensor
initialization advances much slower than wall time. The former
`READINESS_TIMEOUT` environment setting is no longer used. Fresh measured
odometry, flight state, motor telemetry and ready algorithm workers still gate
flight; process failures and a stopped container still fail startup.

Controller settings select `algorithm: mpc` through the small ROS-free algorithm
factory. This uses the existing MPC implementation and unchanged upstream
solver/dynamics; see [Development](DEVELOPMENT.md#architecture). All four presets
retain their full three-/five-UAV fleets and the existing CLI. There is no legacy
backend selection or one-UAV migration stage.

### Scen1: five UAVs in a 1000 × 1000 m map

`./scripts/dev mpc --preset scen1 --record-video` selects `config/fleet-5uav.yaml`,
`scenarios/scen1.yaml`, and `config/controller-scen1.yaml`. Launch derives all five
ground spawns and the `mpc_scen1` world name from the scenario. The upstream
submodule remains unchanged; its 33 rectangles, 32 circles, 27 target waypoints,
and per-leg speeds are frozen into the local scenario, independent of Git at
runtime. Normal target speed is 5 m/s, with the original 13 m/s sprint legs.

Simulation-specific differences from upstream:

- The current starts are `[638.81,354.575,30]`, `[708.72,182.49,30]`,
  `[83.569,101.826,30]`, `[118.523,704.122,30]`, and `[259.686,486.327,30]`.
  UAV 3 initially covers the target; the other four approach from hundreds of
  metres away rather than spawning on formation slots. The frozen target route
  and obstacles remain unchanged. All UAVs take off from ground level under
  native flight-node and Gazebo rotor control.
- `target_startup_ramp_sim: 3.0` ramps the route clock from zero to its nominal
  rate over the first three tracking seconds. On the initial 5 m/s leg this
  gives roughly 1.67 m/s² tangential acceleration and 7.5 m of travel during
  ramp-up, allowing the initially stationary leader to follow. It adds 1.5 s
  to unpaused route duration; later 5/13 m/s legs are unchanged. Camera-loss
  pauses retain ramp progress, and mission reset restarts it. Other presets
  retain their original timing (`target_startup_ramp_sim` defaults to zero).
- Formation logic is unchanged: full cardinal leader-to-follower slots are
  `SLOT_SPACING_RATIO * VIEWING_RADIUS = 1.75 * 25 = 43.75 m`, with obstacle
  contraction and gradual re-expansion. This is not a fixed 50 m separation
  between every pair: adjacent/opposite full follower slots are about 61.9/87.5 m
  apart, and measured tracking/safety constraints also affect actual spacing.
- Flight altitude is 30 m; obstacle collision height is 40 m, so planar
  avoidance remains necessary. The stabilized 320 × 320 cameras use an HFOV
  calibrated to a 25 m half-footprint at ground level. The target marker has
  a 1 m half-width. LiDAR range is 50 m with 360 samples at 10 Hz.
- Only this preset sets `coverage_ray_diameter` and `coverage_outline_diameter`
  to 0.30 m so the render-only guides remain visible from the high observer.
  Both the server recording and GUI use these widths; other presets retain
  0.018/0.035 m. These guides remain excluded from UAV cameras, LiDAR, shadows,
  and collision geometry. Rebuild native plugins after changing rendering code.
- The following observer offset is `[-100,-100,260]` m, with automatic far-clip
  extension. It frames the local fleet and surrounding obstacles, not the
  entire kilometre-wide field. The occupancy panel shows the whole field.
- The local controller overrides the scenario's 10 m/s and 5 m/s² prediction
  bounds with 6 m/s and 3 m/s², a 20-step nominal horizon and a 30-step stopping
  branch at 0.1 s. Its predictive acceleration lag is 0.3 s. These are numerical
  model settings, not a measured native response envelope; slowing physics does
  not correct a prediction/rotor-dynamics mismatch. Optional speed recovery
  remains enabled, and prediction bounds are not strict measured-motion limits.
  Native takeoff/landing use measured state and the configured vertical speed
  limit; landing retains a 120 simulation-second deadline.
  Evaluation excludes low-altitude takeoff/landing using the lower of
  `flight_altitude - 0.5` and `0.9 * flight_altitude`, rather than a fixed 2.5 m
  threshold. This permits measured-versus-truth altitude error without calling
  early climb records a representative 30 m flight.
- A 2 m grid has 501 × 501 cells per worker, versus 668 × 668 cells for the
  existing 200 m / 0.3 m preset. Local planning remains bounded to 150 m
  half-windows (at most 151 × 151 cells). Explicit-ray integration, log-odds
  clipping, dirty inflation, and corridor surface extraction use only the
  touched/local region. Full persistent maps and coordinator snapshots still
  scale with grid cell count; they are not sparse or unbounded-resolution maps.
- Decorative building meshes cap floors and window columns at 24 each per
  facade, rather than generating sub-metre windows across every 40–60 m wall.
  Existing small-field buildings retain their detail. Collision shapes and
  LiDAR proxies are unchanged; decorative city heights can exceed collision
  height, as in the original presets.
- Scen1 warms exact corridor sizes 4–12 and 16 before flight using
  `CORRIDOR_CACHE_FACES`; other presets retain sizes 4–8 and 16. Unusual sizes
  still retain every face in the existing two-entry LRU reserve. This moves
  anticipated solver construction out of the flight deadline, not out of
  overall startup cost or resident memory. IPOPT has a 0.75 s process-CPU cap,
  and each worker phase has a 2 s wall deadline. Independent 0.4 s simulation-time command
  expiry is unchanged and can reject late results before that deadline.
- Inter-UAV safety/soft avoidance distances are 4/10 m, and slot expansion is
  limited to 2 m/s so re-expansion leaves speed headroom behind a 5 m/s target.
- Physics pacing uses `simulation_real_time_factor` from the selected settings
  to provide compute headroom without
  changing simulation-time control periods, trajectory stamps, expiry, or
  ownership/landing guards. Worker and CPU caps remain finite, and failures
  remain visible in telemetry.

The 13 m/s sprints exceed the UAV prediction bound deliberately; they are a
handoff stress case, not a guarantee of continuous visibility. Inspect camera
observations, leader changes, solver failures, measured clearance and actual
mission duration before treating a longer or full-route run as successful.

Native run `20261105T073226Z-7527` completed 90 tracking simulation seconds with
all five UAVs, camera perception and recording, followed by settled landing and
measured rotor shutdown. UAV 3 retained camera tracking (maximum observation age
0.126 s); four nominal solves exceeded the existing CPU cap, remaining visible
as partial failures. Minimum sampled obstacle body clearance was 1.203 m.
This bounded run does not validate the complete route or later 13 m/s sprints;
earlier backend operating envelopes do not apply. See the native verification
record in [Development](DEVELOPMENT.md#native-migration-verification).

### Five UAVs in a 200 × 200 m map

`./scripts/dev mpc --preset large200_5uav --record-video` selects these matching inputs:

- [config/fleet-5uav.yaml](../config/fleet-5uav.yaml): five native multicopters, each with its own ROS flight process and Gazebo safety plugin.
- [large200_5uav.yaml](../ros_ws/src/solo_mpc/scenarios/large200_5uav.yaml): 200 × 200 m bounds, five starts, four cross-shaped formation slots, 240 random rectangles, 240 random circles, and a smooth route with early slalom bends.
- [controller-large200.yaml](../ros_ws/src/solo_mpc/config/controller-large200.yaml): 3 m altitude, 6 m LiDAR range and a 0.35 s solver CPU cap. The original large scenario supplies its 2.2 m/s MPC speed bound; target speed is 1.0 m/s. Actual UAV speed varies through turns and formation contraction and can briefly overshoot the prediction bound. Worker deadlines, expiry and supervision remain enabled.

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

### Separated five-UAV rendezvous

`./scripts/dev mpc --preset large200_5uav_separated --record-video` selects
[large200_5uav_separated.yaml](../ros_ws/src/solo_mpc/scenarios/large200_5uav_separated.yaml)
with the same five-UAV fleet and large-map controller. It preserves the original
200 × 200 m obstacle field, later route waypoints and 3 m flight altitude.
UAV 1 and the first target waypoint shift 2 m northeast on each axis to `[18,18]`.
The scenario sets `params.VMAX: 2.2` m/s and `params.UMAX: 3.0` m/s², increased
from 1.2 m/s and 2 m/s². The target moves at 0.6 m/s on its first two legs to
allow rendezvous, then 1.0 m/s (previously 0.4/0.6 m/s). The shared large-map
controller leaves both bounds to the scenario/defaults; the original
`large200_5uav` scenario and generator retain 2.2 m/s and the default 2 m/s².
Controller overrides still take precedence. The target follows a time-sampled
spline with per-leg speeds; it has no configurable acceleration bound.
These prediction bounds do not guarantee strict measured-speed limits or shorter
solve times. The four follower starts are:

| UAV | Start XYZ (m) | Initial distance from UAV 1 |
|---|---|---:|
| UAV 1 | `[18,18,3]` | 0 m |
| UAV 2 | `[38,2,3]` | 25.61 m |
| UAV 3 | `[10,14,3]` | 8.94 m |
| UAV 4 | `[8,50,3]` | 33.53 m |
| UAV 5 | `[48,40,3]` | 37.20 m |

UAV 3 starts southwest of UAV 1 to approach the west formation slot
from the same side, reducing crossing with the northern arrivals. Its initial distance is deliberately
shorter than the other followers; slot assignment remains dynamic.
UAV 1 starts directly above the stationary pink target at `[18,18]`, centered
in its stabilized downward camera. Takeoff completes before target motion starts.
The default camera mode exercises actual marker perception; do not select
`--ground-truth-target` when checking initial visual acquisition and retention.
Followers start outside camera/formation range and use existing SEARCH pursuit,
local sensor maps and MPC to approach the shared target estimate. Existing
leader-distance hysteresis then switches them to TRACK and assigns formation
slots. Leadership remains selected by observed visibility; it is not forced to
UAV 1. Run long enough for the distant followers to rendezvous before assessing
team tracking. The scenario is a separate checked-in snapshot: regenerating
`large200_5uav.yaml` does not update its obstacle field or route automatically.

Native run `20261105T075748Z-16619` completed 30 tracking simulation seconds with
all five UAVs, camera input and recording. All followers entered TRACK by the
end; all 300 fleet cycles were ready, with no failed nominal solves. Minimum
sampled obstacle body clearance was 0.456 m and inter-UAV distance was 1.227 m;
all five subsequently landed and stopped their rotors. Prediction bounds and
this bounded mission do not establish hard physical limits, full-route success,
repeatability or continuous collision guarantees.

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
names match `[a-z][a-z0-9_]*` and are unique. Each `spawn` is
`[x,y,z,roll,pitch,yaw]`, with angles in degrees. MPC launch replaces fleet spawn
positions with scenario XY and a 0.195 m ground height; flight altitude is
controlled separately. Names identify native model/topic namespaces, not network
endpoints. Vendored original Iris models, four rotor motor systems and native
velocity control are the only vehicle backend.

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
Gaussian range-noise standard deviation. Each UAV integrates its own remaining
valid rays into its persistent occupancy grid before MPC uses the accumulated
maps. There is no minimum valid-ray fraction. A fresh scan with no usable rays
leaves that UAV's map unchanged and does not block MPC or another UAV's mapping.
Scan age and timestamp-matched measured poses are still required.

Projection uses full measured roll, pitch and yaw, including the rotated 0.08 m
LiDAR mount offset. There is no endpoint-height band around the sensor. Rays
ending at or below the known world ground plane (Z=0, with a 1 mm tolerance)
are discarded entirely, so they cannot clear cells. NaN, negative infinity,
zero, below-minimum and finite above-maximum ranges are also discarded.
Positive infinity and maximum-range no-returns clear only their observed XY
prefix; finite in-range hits mark their projected endpoint occupied. Teammate
returns retain their free prefix without marking the teammate occupied.

This remains a 2D projection: upward/downward rays observe different heights
from the planned 3 m flight plane. A ray can pass above a low obstacle and clear
its XY footprint, or project an overhang/upper surface into the flight corridor.
The numerical ground tolerance does not bound Gaussian range noise; a noisy
ground return reconstructed above it can remain as a false occupied cell.
Sparse/empty scans add no evidence in unseen directions, and persistent maps can
retain old evidence. An optimistic map is not proof that unobserved space is clear.

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

`simulation_real_time_factor` defaults to **0.2** in the supplied controllers
and when omitted from custom settings: one simulation second takes at least five
wall seconds. Set it in the controller YAML selected by `--settings`, then restart
the stack. Use `0.1` for more compute headroom or `1.0` for real-time pacing.
The value must be finite and positive. Gazebo caps the requested rate; CPU/GPU
load can make the observed rate lower. Inspect `status/health.json`'s
`simulation.real_time_factor` and the evaluation report for the actual rate.

This changes wall pacing, preserving the 1 ms physics step, vehicle/target
speeds in m/s, prediction timestep, sensor rates, and 20 FPS video timeline.
At 0.2×, the 0.1 s control period allows about 0.5 wall seconds per cycle;
a 250 ms solve consumes about 50 ms of simulation time. It does not reduce solver
CPU cost or guarantee collision avoidance under arbitrary load. Choose a rate
below `control_period / observed cycle wall time`, allowing scheduling headroom.

Gazebo is the clock authority. Native velocity control and motor dynamics advance
with physics. ROS planning, target, frames, sensors, flight/mission supervision
and setpoints use `/clock`. `OdometryPublisher` provides world ENU pose and body
twist on `/uavN/ground_truth/odometry`; `frame_node` rotates twist into world ENU
and retains source stamps rather than stamping receipt time. Joint trajectories
retain their original source epoch through sampling and expiry. The independent
sampler emits current-tick world `/uavN/flight/velocity` requests only while the
original trajectory is fresh. The recorder samples original simulation
timestamps at 20 FPS, so saved playback retains normal physical speed.
Live GUI motion is slower and collecting the same video duration takes longer.

The native sampler combines sampled planned velocity, measured-position error
(`position_kp`, default 1.0/s), and planned acceleration multiplied by
`acceleration_feedforward_time` (default 0.22 s). The latter compensates the
native horizontal velocity loop's mass/gain ratio (1.76 kg / 8), retaining the
acceleration intent when every MPC horizon restarts at measured velocity.
Both gains must be finite and nonnegative. The resulting horizontal request is
still speed-limited, proximity-guarded, independently expired, and realized only
by attitude/rotor forces; predicted positions never set the vehicle pose.
Altitude uses separate measured-height feedback and the vertical speed limit.

`mission run --seconds N` and `mission start --seconds N` limit tracking to `N`
simulation seconds on Gazebo's `/clock`, independently of the requested or
observed real-time factor. The limit starts after takeoff when control and target
motion are enabled. Omit `--seconds` or set it to zero for the complete route;
goal arrival or safety handling can end tracking sooner. Startup, takeoff, and
landing are outside this limit. Recording continues until the stack stops, so
takeoff/landing and any post-mission wait can make the final 20 FPS video longer
than `N` seconds. CLI elapsed progress uses wall seconds;
safety watchdogs keep their independent wall-time budgets.

| Setting | Default |
|---|---:|
| `algorithm` | `mpc` |
| `simulation_real_time_factor` | 0.2 (requested simulation seconds per wall second) |
| `flight_altitude` / `obstacle_height` | 3 / 5 m |
| `control_period` / `publish_rate` | 0.1 s / 20 Hz |
| `controller_execution` | `parallel`; `sequential` orders the same persistent workers |
| Worker phase / startup timeout | 1 / 30 wall seconds |
| State / sensor freshness | 0.5 / 0.5 simulation seconds |
| Target freshness / search budget | 1.5 / 8 simulation seconds |
| Persistent planning failure budget | 3 simulation seconds |
| Command expiry | 0.4 simulation seconds and 3 wall seconds |
| Pose wall freshness | 3 seconds |
| Flight/motor freshness | 2.5 simulation seconds and 3 wall seconds |
| Final fleet arrival radius / timeout | 6 m / 30 simulation seconds |
| `landing_timeout_sim` | 60 simulation seconds to confirm settled ground and motor shutdown |
| `landing_stall_timeout_wall` | 60 wall seconds without clock progress or flight state |
| `diagnostic_capture_cycles` | 128 retained calls per UAV; 0 disables capture |

Timeouts bound failures; they do not guarantee a 100 ms fleet cycle. Native
planner kernels warm both contiguous and sliced windows before worker readiness.
See [Development](DEVELOPMENT.md#frames-and-clocks) before changing clock policy.

Worker deadlines, IPOPT CPU caps, pose/flight wall freshness, command wall expiry,
and landing stall watchdogs remain independent wall/CPU limits; they are not
multiplied by the slowdown. Source timestamps and simulation expiry remain
unchanged. Extreme slowdowns can therefore trip a wall freshness guard; inspect
the recorded reason rather than disabling protection. Evaluation retains wall
`compute_*` and `over_100ms_fraction` and separately reports
`delivery_delay_sim_p50_p95_p99_max_ms` (input-to-result delay on `/clock`) and
`delivery_over_control_period_fraction` against the configured simulation period.

The mission supervisor imposes no wall-time timeout on target simulator updates.
Target publication follows Gazebo time and successful pose requests, so a slow
CPU can delay updates without aborting the mission. Target freshness and camera
SEARCH limits use simulation time. Pose wall freshness still guards vehicle
setpoint publishing; command expiry, controller health, and flight checks also apply.

The per-UAV flight node checks source simulation age and receipt wall age before
accepting world velocity requests, then exclusively emits body `/cmd_vel` and
`/enable`. Separately, C++ `SoloFlightSafety` checks simulation/wall command expiry
inside Gazebo, descends on command/process loss and disables motors only after
settled ground. Inspect actual `/uavN/motor_speed` (`actuator_msgs/Actuators`) and
`/uavN/safety_active`; a requested disable alone does not prove motor shutdown.
Owner-token services and measured landing confirmation remain mandatory.

No new CPU/performance acceptance thresholds are introduced by the migration.
Preserve existing budgets and visible failures; new native results for every
full-fleet camera/recording preset will be supplied by the parent task. Restart
after changing inputs; release source changes require a new image.
