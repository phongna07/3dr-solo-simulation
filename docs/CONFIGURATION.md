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
`SIM_IMAGE`,
`ROS_DOMAIN_ID` (42), `BUILD_JOBS` (4), and `LIBGL_ALWAYS_SOFTWARE`.
The runner supplies host UID/GID and a fresh run ID. Select GPU/display overrides
as described in [Simulation](SIMULATION.md#gpu-and-display).

Readiness waits indefinitely, including on slow hosts where FCU initialization
advances much slower than wall time. The former `READINESS_TIMEOUT` environment
setting is no longer used. FCU heartbeat and pose freshness checks still apply;
startup process failures and a stopped container still fail startup.

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
  SITL control.
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
  branch at 0.1 s. Its effective acceleration lag is 0.3 s, with SITL
  `PSC_JERK_XY: 20` m/s³. ArduCopter's default 5 m/s³ horizontal input shaping
  delayed braking during saturated SEARCH approaches; recorded response lag
  was about 0.7 s. Keep input shaping and the prediction settings together:
  slowing physics alone does not correct the braking response. Optional speed
  recovery remains enabled; prediction
  bounds are not strict measured-motion limits. SITL horizontal navigation
  speed/acceleration are 12 m/s and 5 m/s²; climb is 3 m/s and final descent is
  1 m/s. Landing retains a 120 simulation-second deadline.
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
- Physics runs at at most 0.1× real time to provide compute headroom without
  changing simulation-time control periods, trajectory stamps, expiry, or
  ownership/landing guards. Worker and CPU caps remain finite, and failures
  remain visible in telemetry.

The 13 m/s sprints exceed the UAV prediction bound deliberately; they are a
handoff stress case, not a guarantee of continuous visibility. Inspect camera
observations, leader changes, solver failures, measured clearance and actual
mission duration before treating a longer or full-route run as successful.

The dispersed starts and current flight settings were checked in
`runs/20261105T033654Z-12568/`: 30 simulation seconds of camera tracking completed,
followed by observed disarm of all five UAVs. Sampled Gazebo body clearance was
at least 0.99 m. No controller delivery exceeded the 0.1 s simulation-time
control period; five CPU-capped solves (two for UAV 4, three for UAV 2) recovered.
UAV 3 retained tracking while the distant followers remained in SEARCH; this
does not verify formation joining or the full route. Source hashes matched
the pinned upstream revision. See `reports/verification.json`,
`reports/mission-completed.json`, `reports/evaluation.json`, and the recording.
Earlier evidence below uses earlier starts and controller settings.

Verification evidence (run IDs are UTC):

- `runs/20261105T014442Z-8991/`: current separated starts, 3 s startup ramp,
  and thicker guides; 30 s camera tracking completed with UAV 1 retaining
  leadership throughout and all 1,500 tracking solves nominal. During the first
  6 s, the target retained at least 10.79 m of margin inside UAV 1's true camera
  footprint; maximum observation age was 0.125 s. All followers joined TRACK
  within 3.9 s; final-five-second median leader distances were 42.6–44.7 m.
  All five landed/disarmed. Minimum body clearance/separation were 18.89/6.09 m.
  Both native plugins rebuilt; the recording visibly shows the wider guides,
  while installed-package inspection confirms unchanged defaults for other
  presets. See `reports/separated-start-verification.json`,
  `package-inspection.json`, `coverage-config.json`, and `frames/`.
- `runs/20261104T163129Z-4514/`: 90 s ground-truth baseline from the
  then-current cross start, before the final cache/safety/slot-expansion tuning,
  at 0.2×.
  Tracking completed; all five UAVs landed/disarmed. Minimum measured body
  clearance was 4.54 m and separation 5.56 m. One nominal solve hit its CPU cap.
- `runs/20261104T164739Z-8377/` and `runs/20261104T170529Z-13780/`: faster
  0.4× / 0.2× camera experiments ended in supervised LAND after worker deadlines
  under heavy unrelated host CPU load. All vehicles subsequently disarmed.
  `20261104T170416Z-8011` was a separate, unarmed MAVROS startup crash.
- `runs/20261104T172212Z-31938/`: pre-separated-start settings, 60 s camera mission through
  the first 13 m/s sprint. The recorded `config/scenario.yaml` is a route-section
  override: first three waypoints/legs omitted, with clear starts at waypoint 4;
  obstacle geometry and remaining speeds are unchanged. UAV 5 took leadership
  from UAV 1, briefly entered SEARCH, and reacquired the target. Tracking
  completed and all five vehicles landed/disarmed. Minimum body clearance was
  2.54 m; separation was 3.90 m, slightly below the 4 m prediction/guard setting.
  Nominal solves succeeded in 96.3% of per-UAV samples: 117 CPU-cap failures and
  one infeasible solve remain visible; only 85.9% of fleet samples were fully
  ready. Mapping p95 was 2.45 ms per UAV under that load. This is a successful
  bounded mission, not a zero-failure or full-route validation.

Each flight directory retains `reports/evaluation.json`, `mission-cli.log`,
`scen1-verification.json` where present, telemetry, optimizer captures, source
provenance and the finalized `recorded-mission.mp4`; later checks also retain
`reports/mission-final.json` before normal stop overwrote live mission status.
The unchanged solver/dynamics hashes matched pin `3f71d4c` in all flight runs.
The full approximately 750 s ramped route and later sprints remain unverified. Do not
increase pacing on a busy host based solely on successful command exits.
`./scripts/dev build sim` also completed; an unmounted, non-development container
loaded the installed preset and bundled upstream sources with matching hashes.
That packaging inspection is recorded in `reports/installed-package-check.log`
and is not an installed-image flight mission.

### Five UAVs in a 200 × 200 m map

`./scripts/dev mpc --preset large200_5uav --record-video` selects these matching inputs:

- [config/fleet-5uav.yaml](../config/fleet-5uav.yaml): five independent SITL/MAVROS instances.
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

Verified on November 4, 2026 with the default camera/LiDAR inputs:

```bash
./scripts/dev mpc --preset large200_5uav_separated --record-video
./scripts/dev mission run --seconds 90
./scripts/dev evaluate
./scripts/dev stop
```

Run `runs/20261104T155857Z-2649/` completed 90 simulation seconds of target
motion; all five UAVs entered TRACK within 17.53 s, then landed and disarmed.
Sampled minimum obstacle-body clearance was 0.429 m and inter-UAV separation
1.219 m; nearest-UAV target distance was 1.237 m at p95. One UAV 4 solve hit
the unchanged 0.35 s CPU cap and recovered on the next cycle (4,664 successful
solves of 4,665). Maximum source-to-delivery delay was 96 ms in simulation
time, below the 100 ms control period. Wall-time compute p95 was 165.7 ms;
the host achieved 0.274× real time despite the controller requesting 1×.
Measured horizontal speed peaked at 2.875 m/s, exceeding the prediction bound;
the existing measured-state speed recovery remained enabled. Upstream source
hashes matched the pin. See `reports/evaluation.json`, `reports/speed-tuning.json`,
`reports/mission-completed.json`, and the finalized `recorded-mission.mp4` in
that run. The first increased-limit configuration completed without further
tuning; this bounded mission does not establish full-route or repeatability
success, hard physical limits, or collision guarantees.

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

`simulation_real_time_factor` defaults to **0.2** in both supplied controllers
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

Gazebo is the clock authority. Each FCU receives Gazebo timestamps through the
JSON plugin with `lock_step: 1` and `no_time_sync: 1`; SITL stays at `--speedup 1`
because it must not apply a second wall-rate pacing loop. ROS planning, target,
frames, sensors, mission supervision, and setpoints use `/clock`. MAVROS keeps
wall time with MAVLink timesync and SYSTEM_TIME disabled; the frame adapter
stamps FCU measurements at simulation receipt time. The recorder samples original
simulation timestamps at 20 FPS, so saved playback retains normal physical speed.
Live GUI motion is slower and collecting the same video duration takes longer.

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
| FCU freshness | 2.5 simulation seconds and 60 wall seconds |
| Final fleet arrival radius / timeout | 6 m / 30 simulation seconds |
| `landing_timeout_sim` | 60 simulation seconds to confirm disarm |
| `landing_stall_timeout_wall` | 60 wall seconds without clock progress or FCU state |
| `diagnostic_capture_cycles` | 128 retained calls per UAV; 0 disables capture |

Timeouts bound failures; they do not guarantee a 100 ms fleet cycle. Native
planner kernels warm both contiguous and sliced windows before worker readiness.
See [Development](DEVELOPMENT.md#frames-and-clocks) before changing clock policy.

Worker deadlines, IPOPT CPU caps, pose/FCU wall freshness, command wall expiry,
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
setpoint publishing; command expiry, controller health, and FCU checks also apply.

Resolved `sitl_params` default to `PLND_ENABLED: 0`: this simulation uses normal
LAND and supplies no IRLOCK precision-landing observations. Local overrides are
written to `config/sitl-overrides.parm` after the unchanged Iris defaults;
explicit `sitl_params` entries take precedence. Restart the stack after changing
these settings. A release deployment requires a new image for source changes.
