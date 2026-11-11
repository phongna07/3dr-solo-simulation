# Configuration

[Overview](../README.md) · [Simulation](SIMULATION.md) · [Development](DEVELOPMENT.md)

Stop the simulation before changing inputs. Launch snapshots configuration;
editing a YAML file does not reconfigure an active experiment.

## Input files

| Input | Default | Purpose |
|---|---|---|
| Fleet | [config/fleet.yaml](../config/fleet.yaml) | UAV names and native Gazebo spawn poses |
| Scenario | [scen5_1000.yaml](../src/solo_mpc/scenarios/scen5_1000.yaml) | Bounds, starts, obstacles, target route, algorithm parameters |
| Controller | [controller-scen5_1000.yaml](../src/solo_mpc/config/controller-scen5_1000.yaml) | Altitude, sensors, execution, timing, parameter overrides |

Use container paths for CLI overrides:

```bash
./scripts/dev run --fleet /workspace/config/fleet-custom.yaml   --scenario /workspace/config/scenario-custom.yaml   --settings /workspace/config/controller-custom.yaml
```

`.env` supports `FLEET_CONFIG`, `MPC_SCENARIO`, `MPC_SETTINGS`,
`SIM_IMAGE`,
`ROS_DOMAIN_ID` (42), `BUILD_JOBS` (4), and `LIBGL_ALWAYS_SOFTWARE`.
The runner supplies host UID/GID and a fresh run ID. Select GPU/display overrides
as described in [Simulation](SIMULATION.md#gpu-and-display).

Readiness waits indefinitely, including on slow hosts where native model/sensor
initialization advances much slower than wall time. The former
`READINESS_TIMEOUT` environment setting is no longer used. Fresh measured
odometry, flight state, motor telemetry and ready algorithm workers still gate
flight; process failures and a stopped container still fail startup.

Controller settings select a registered `algorithm` (default `mpc`) through the
small ROS-free algorithm registry; `./scripts/dev run --algorithm NAME`
(`MPC_ALGORITHM`) overrides it for one run. `mpc` uses the existing MPC
implementation and unchanged upstream solver/dynamics; `apf` runs the unchanged
upstream artificial potential field baseline on the same leader/slot layers;
`cbf_qp_lag` runs the unchanged upstream safety barrier certificate QP filter
in its lag-aware mode on the same layers; `pure_mpc` runs the
unchanged upstream plain MPC baseline on the same layers; `impc_ob_rec` runs
the unchanged upstream IMPC-OB baseline with its LiDAR recovery variant; see
[Development](DEVELOPMENT.md#baseline-algorithms). Both presets
retain their full three-/five-UAV fleets and the existing CLI. There is no legacy
backend selection or one-UAV migration stage. Explicit `--fleet`, `--scenario`,
and `--settings` arguments after `--preset` override that preset; the original
no-argument defaults remain unchanged.

### Scen5_1000: unscaled three-UAV upstream hand-off layout

`./scripts/dev run --preset scen5_1000 --record-video` selects `config/fleet.yaml`,
`scenarios/scen5_1000.yaml` and `config/controller-scen5_1000.yaml`. This is the
default. The scenario is upstream `scenarios/scen5.yaml` verbatim plus
`world_name: mpc_scen5_1000`: 1000 × 1000 m, the three active upstream starts at
30 m (its description still says 5 UAVs), 62 rectangles, 57 circles, 61 waypoints,
six 13 m/s sprint legs (else 5 m/s) that force leader hand-offs, 25 m
half-footprint, `CBF_BOX_RATIO` 0.5, 20-step horizon, VMAX 10 m/s, UMAX 5 m/s² and
2 m grid. Fleet spawns are those starts on the ground.

Controller values, simulation-only values and known differences from upstream
are those of `scen1_1000` below. It supplies the same four-slot `GRID_CELLS`.

### Scen1_1000: unscaled five-UAV upstream layout

`./scripts/dev run --preset scen1_1000 --record-video` selects
`config/fleet-5uav.yaml`, `scenarios/scen1_1000.yaml` and
`config/controller-scen1_1000.yaml`. The scenario is the upstream
`scenarios/scen1.yaml` verbatim plus `world_name: mpc_scen1_1000`: 1000 × 1000 m,
original starts at 30 m, 33 rectangles, 32 circles, 27 waypoints, 5/13 m/s target
legs, 25 m half-footprint, 20-step horizon, VMAX 10 m/s, UMAX 5 m/s² and 2 m grid.
Controller `params` repeat VMAX because `flight_node` reads it from that file, and
supply upstream's four-slot `GRID_CELLS` default.

Simulation-only values: obstacle collision height 55 m (meshes derive from it and
reach about 69–139 m; 25 m above flight altitude keeps rays below the top out to 50 m
range up to 30° tilt), observer offset `[-70,-70,240]`, coverage guides 0.3 m, camera
HFOV unchanged (30 m altitude gives the 25 m footprint), climb/descent limit 2 m/s.

Known differences from upstream: IPOPT keeps the local 0.75 s CPU cap; the native
velocity controller limits acceleration to 4 m/s² horizontally (`world.py`), below
UMAX; `CORRIDOR_MARGIN` is 1.5 m rather than upstream's derived 0.77 m. The Iris
stays physical size, so it is barely visible in the 240 m observer view; the FOV
guides and panels show each UAV.

Obstacle inputs follow upstream per algorithm. In every preset `mpc` decomposes the
current scan's exact hit points, without memory, as upstream
`generateSafeCorridor`. `apf`, `cbf_qp_lag`, `pure_mpc` and `impc_ob_rec`
take those hits directly. JPS, as upstream, integrates exact hits into the
accumulated grid and searches its cells; that grid also feeds the video occupancy
panel. The 1.5 m margin applies to every algorithm that reads it (not `apf`).

## Fleet and geometry

The default three-UAV scenario starts at `[78.191,378.774,30]`,
`[320.185,761.932,30]`, `[359.173,626.146,30]` in fleet order, facing north (+Y).
The target starts at `[100,350]` and follows a smooth winding route to
`[863.327,643.623]` at **5 m/s**, with four **13 m/s** sprint legs. Horizontal UAV
speed is limited to **10 m/s** by the supplied controller overrides.

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
| `SPEED_RECOVERY` | true | Local adapter recovery for initially overspeed measured states (`mpc`, `pure_mpc`, `impc_ob_rec`) |
| `BACKUP_*_TOLERANCE` | 0.15 m / 0.3 m/s / 0.5 m/s² at `VMAX` 1.2, `UMAX` 2 | Plan-replay agreement; unless overridden, scaled by `VMAX` / `UMAX`, position capped at `CORRIDOR_MARGIN − ROBOT_RADIUS` |
| `IPOPT_OPTIONS` | 500 iterations / 0.2 s CPU | Solver limits; worker deadline remains independent |
| `BASELINE_LEADER_STANDOFF` | `CBF_BOX_RATIO` × `VIEWING_RADIUS` (derived) | Baseline (`apf`, `cbf_qp_lag`, `pure_mpc`, `impc_ob_rec`) leader goal distance from the predicted target |
| `CBF_QP_GAMMA` | 1.0 | Baseline `cbf_qp_lag`: class-K function κ(h) = γh³ (upstream value) |
| `CBF_QP_KP` / `CBF_QP_KD` | 1.0 / 2.0 | Nominal PD u = −KP(p − r) − KD(v − ṙ) to the guidance goal (upstream values) |
| `PURE_MPC_W_COL` | 1.5 | Baseline `pure_mpc`: sigmoid collision cost weight (`W_col` of upstream's original `robot_mpc.py`) |
| `IMPC_EPSILON` | 0.2 L, L = `UAV_SAFE_DISTANCE` / 0.6 (derived; 0.333 m in all presets) | Baseline `impc_ob_rec`: warning-band width (reference 0.2 m for 0.3 m robots) |
| `IMPC_BUFFER` | 0.03 L (derived; 0.05 m in all presets) | Baseline `impc_ob_rec`: obstacle-constraint slack bound; `CORRIDOR_MARGIN − IMPC_BUFFER ≥ ROBOT_RADIUS` is required, `IMPC_BUFFER < IMPC_EPSILON` warned, only for this algorithm |

The supplied controller enables `SPEED_RECOVERY` in the local adapter. It permits
only the initial predicted overspeed that its maximum-effort braking model cannot
remove immediately, then restores the nominal speed bound. Measured state,
corridor and collision constraints, solver CPU cap, and command expiry remain
in effect; the upstream source is unchanged. Omit the override or set it to
`false` to use the adapter's default strict speed bounds.

In SEARCH mode the local controller passes the upstream `W_search_track` pull a
carrot on the planned path, `VMAX` × `HORIZON_LENGTH` × `TIMESTEP` ahead, instead
of the predicted target; the target is used only once the path is shorter than
that. A straight-line pull towards a distant target otherwise pins
UAVs against building faces and drives IPOPT into its CPU cap.

Derived values include inflation radius, world bounds, standoff distance,
formation thresholds, and corridor margin. Unless overridden, `CORRIDOR_MARGIN`
includes body radius, three Gaussian LiDAR-noise standard deviations and ray
spacing (0.77 m with the 1000 m preset parameters, which pin 1.5 m instead). Noise is unbounded. The forward corridor seed
must leave at least one cell of usable range. Keep obstacle collision height
above flight altitude plus body radius.

After a nominal solve fails, a retained stop branch can be replayed only while
measured XY position, velocity, and acceleration agree within 0.15 m, 0.3 m/s,
and 0.5 m/s². Otherwise no fresh reference is produced; independent expiry and
supervision apply. Failed nominal solves remain visible as `nominal_ok: false`.
These sampled/soft constraints do not establish collision guarantees.

## Sensors and target input

Both presets use 320 × 320 cameras at 8 Hz, square images, and approximately 79.8°
horizontal FOV. The ideal Gazebo gimbal keeps the camera downward and world-aligned,
with its optical center 0.04 m below the body. It cancels attitude motion without
mechanical dynamics. The ground footprint is about 50 × 50 m at 30 m body
altitude. UAV cameras are created only with `--record-video`; they feed the
recorded FOV panels and coverage overlays, not target estimation.

Horizontal body-mounted LiDAR uses 360 rays at 10 Hz, 50 m range, and about 0.017 m
Gaussian range-noise standard deviation. Each UAV integrates its own remaining
valid rays into its persistent occupancy grid before path planning and formation
checks use the accumulated maps. There is no minimum valid-ray fraction. A fresh scan with no usable rays
leaves that UAV's map unchanged and does not block MPC or another UAV's mapping.
Scan age and timestamp-matched measured poses are still required.

Projection uses full measured roll, pitch and yaw, including the rotated 0.08 m
LiDAR mount offset. Rays ending at or below the known world ground plane (Z=0,
with a 1 mm tolerance) or at or above `obstacle_height` are discarded entirely,
including returns, so they cannot clear cells. NaN, negative infinity,
zero, below-minimum and finite above-maximum ranges are also discarded.
Positive infinity and maximum-range no-returns clear only their observed XY
prefix; finite in-range hits mark their projected endpoint occupied. Teammate
returns retain their free prefix without marking the teammate occupied.

This remains a 2D projection: upward/downward rays observe different heights
from the planned flight plane. Because all obstacles share `obstacle_height`, a
ray kept below it cannot pass over one, but a tilted ray can still project an
upper or lower obstacle surface into the flight corridor. Discarded rays add no
evidence, so sustained tilt reduces how far each scan observes.
The numerical ground tolerance does not bound Gaussian range noise; a noisy
ground return reconstructed above it can remain as a false occupied cell.
Sparse/empty scans add no evidence in unseen directions, and persistent maps can
retain old evidence. An optimistic map is not proof that unobserved space is clear.

The target node publishes exact timestamped ground-truth positions on
`/target/ground_truth`; the controller consumes them directly, with no camera
detection. Velocity, confidence, and lead pursuit remain estimated. A target
older than `target_timeout_sim` fails the mission with landing cleanup.

`SEARCH` means target pursuit; a distant UAV can be in SEARCH even with fresh
external input. Formation membership still uses viewing-distance gates.
`target_input_fresh` is reported in logs and status.

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
`acceleration_feedforward_time` (default 0.30 s). The latter is the inverse of
the native horizontal velocity loop's fitted gain (3.37/s, measured from flown
commands, below the nominal 8 / 1.76 kg), retaining the
acceleration intent when every MPC horizon restarts at measured velocity.
Sampled velocity is also scaled by `1 + velocity_drag_feedforward` (default 0.04,
the fitted rotor-drag/velocity-gain ratio 0.136/s over 3.37/s). Without it the
proportional loop settles where small planned accelerations balance rotor drag,
which held `pure_mpc` near 3 m/s.
The world generator sets the native horizontal `maximumLinearAcceleration` to
`UMAX` on each axis, so the planned acceleration bound is reachable in any heading.
All three gains must be finite and nonnegative. The resulting horizontal request is
still speed-limited, independently expired, and realized only
by attitude/rotor forces; predicted positions never set the vehicle pose.
Altitude uses separate measured-height feedback and the vertical speed limit.

`run --seconds N` limits tracking to `N`
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
| `algorithm` | `mpc` (or `apf`, `cbf_qp_lag`, `pure_mpc`, `impc_ob_rec`; `--algorithm` overrides) |
| `simulation_real_time_factor` | 0.2 (requested simulation seconds per wall second) |
| `flight_altitude` / `obstacle_height` | 3 / 5 m |
| `control_period` / `publish_rate` | 0.1 s / 20 Hz |
| `solver_lockstep` / `solver_lockstep_hold_wall` | false (true in presets) / 2.5 s | Pause Gazebo during each fleet solve; the setpoint process owns the pause and resumes at the wall deadline, which must lie between the worker timeout and the 3 s wall expiries |
| `solver_compute_delay` | false | Requires `solver_lockstep`: after each paused solve, simulate the measured fleet-cycle time on the previous plans, then deliver (see below) |
| `abort_on_planner_failure` | true (false in presets) | Whether solver fallbacks count toward `planner_failure_timeout_sim`; missing inputs and controller errors always do |
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

With `solver_compute_delay`, each planning request observes at a paused
simulation time `t`, measures the whole fleet cycle (mapping, solves and worker
IPC) in wall seconds `d`, rounded to the 1 ms physics step, and publishes the
plans still stamped `t`. The setpoint process keeps them pending and requests
Gazebo `run_to_sim_time = t + d`; physics, target, sampler and flight nodes keep
running on the previous plans. At `t + d` it delivers the new plans, samples them
`d` into their horizon, and resumes the world. Every UAV shares the fleet delay.
The next observation is at `max(t + control_period, t + d)`. A delay beyond
`command_timeout_sim` expires the previous plan, which appears as `expired_command`;
this is reported, not prevented. Delivered plans take their wall receipt time at
delivery. `compute_delay_sim` and `available_stamp` appear in `telemetry/mpc.jsonl`;
`delivery_delay_sim_*` then measures `available_stamp - stamp`, and the lockstep
counters add `releases` and `expired_releases` (the wall deadline is the hold wall
plus the delay at the requested real-time factor).
The mission supervisor's controller-status wall freshness is extended by the same
pending delay, since the controller reports before Gazebo simulates it.

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
