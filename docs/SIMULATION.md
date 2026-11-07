# Simulation

[Overview and quick start](../README.md) · [Configuration](CONFIGURATION.md) · [Development](DEVELOPMENT.md)

Run commands from the project root. Initialize the submodule before building:
`git submodule update --init --recursive`. Keep your existing `.env`; create it
from `.env.example` only for a new checkout.

## Run a mission

```bash
./scripts/dev build sim
./scripts/dev doctor
./scripts/dev mpc --record-video
./scripts/dev mission run --seconds 30
./scripts/dev evaluate
./scripts/dev stop
```

`mpc` starts a fresh stack and waits for native models, per-UAV flight processes,
measured odometry/motor telemetry, sensors and algorithm workers. Motors stay
disabled. Takeoff waits for world-frame state, ownership, fresh inputs and a
stable fleet altitude
before enabling target motion and control. Startup can take several minutes
with slow rendering. `--seconds` limits tracking using Gazebo's simulation clock,
excluding startup/takeoff/landing; omit it or use `--seconds 0` to attempt the
complete route. Reaching the route goal or a safety stop can end tracking earlier.
While tracking, `mission run`/`start` progress reports elapsed over requested
tracking simulation seconds (`tracking: 12.3/30s sim`) when `--seconds` is
positive, otherwise scenario waypoints the target has reached, counting its
start, over all route waypoints (`target waypoints: 4/12`). The completed,
stopped or failed line repeats the final value.

Physics defaults to **0.2× real time** to give MPC more wall time between
simulation steps. `--seconds 30` requests 30 simulation seconds of tracking,
requiring at least 150 wall seconds at that rate, plus takeoff and landing.
Rendering or CPU load can increase the wall duration without shortening the
requested simulation duration. CLI elapsed progress remains labeled in wall
seconds. Configure `simulation_real_time_factor` in the controller YAML;
see [timing and expiry](CONFIGURATION.md#timing-and-expiry). Recorded playback
retains normal target/UAV speeds at the existing 1920 × 1080 / 20 FPS.
The exported video also includes recorded takeoff/landing and keeps recording
until the stack stops, so its total duration can exceed `--seconds`. Compare its
timestamp sidecar and tracking interval, not the host command's elapsed time.
Normal `stop` finalizes the video.

`telemetry/flight-<uav>.jsonl` records native flight mode/ownership, measured
world position/velocity, source stamps, actual four-rotor speeds in rad/s,
motor-enable state, settled-ground confirmation and safety faults. Confirm
fresh zero rotor speeds and `landed: true`, `armed: false`, `enabled: false`
for every vehicle after the mission, before stopping physics.

For the five-UAV 200 × 200 m random field, use the matching preset:

```bash
./scripts/dev mpc --preset large200_5uav --record-video
./scripts/dev mission run --seconds 300
./scripts/dev evaluate
./scripts/dev stop
```

For the scaled upstream layout in a 200 × 200 m field at 6 m flight altitude:

```bash
./scripts/dev mpc --preset scen1 --record-video
./scripts/dev mission run --seconds 90
./scripts/dev evaluate
./scripts/dev stop
```

Target input is always exact ground truth; UAV cameras only feed recordings.
The complete route is much longer than this representative check and contains
2.6 m/s sprint legs. A duration-limited mission is not evidence of completing
the route or all handoffs. Retain the actual run ID and inspect the evaluation,
per-UAV solver/worker timing, target freshness, landing and finalized video.
Budget wall time from the selected pacing and observed clock rate, plus
takeoff/landing. Recording temporarily retains raw video until finalization;
check free disk space before long missions.

The original large preset's route takes about 464 simulation seconds, with
several early slalom bends through nearby obstacles: at least 39 wall minutes at
0.2×. Consult the actual clock rate.
A time-limited run covers only part of it; rendering five sensors and a large
obstacle field can make simulation slower than wall time. Use the observed clock
rate when choosing a verification duration. Select `--preset small50_3uav` to
return to the original configuration; see [Configuration](CONFIGURATION.md#five-uavs-in-a-200--200-m-map)
for regeneration and parameter details.

| Command | Purpose |
|---|---|
| `mpc --record-video` | Record overhead, map, state instruments, and all UAV FOVs |
| `mpc --gui` / `gui` | Attach Gazebo viewer; closing it leaves simulation running |
| `mpc --no-wait` / `wait` | Start asynchronously / wait indefinitely for readiness |
| `mission start --seconds 30` | Return after tracking starts; supervisor continues |
| `mission status` | Mission state, ownership, errors, and controller diagnostics |
| `mission stop` | Pause target, disable publishing, land owned UAVs, await settled ground and motor shutdown |
| `mission reset` | Reset the target while idle |
| `status` / `logs` / `shell` | Health / process logs / container shell |

Prefix the commands above with `./scripts/dev`. Only one simulation should run
at a time. Native models and ROS nodes share one container; host ROS discovery
and vehicle network endpoints are unnecessary. Use `shell` to inspect topics.

## Stop and recover

Ctrl+C in `mission run` requests cleanup. Confirm landing and disarm before
restarting. The supervisor remains responsible after a `mission start` client
exits. Target loss, stale inputs, persistent controller failures, or flight
violations trigger cleanup.

Normal `./scripts/dev stop` requests mission cleanup before stopping the container.
If landing fails, inspect flight state, actual motor speeds, safety status and
owner-token service errors, then retry `mission stop`.
Failed `./scripts/dev mission run` commands automatically request mission cleanup,
then gracefully stop the container so the recorder finishes encoding and saves
`recorded-mission.mp4`. The container also stops if the cleanup request fails
because physics, flight state, or the supervisor is unavailable. The original
nonzero command status is retained. Before retrying cleanup, the CLI saves the
available mission status to `status/mission-failure.json`; `status/mission.json`
then describes the final cleanup attempt. Inspect both and the process logs.
Successful missions still need an explicit `stop` to finalize the video.
Other mission subcommands keep the stack available on failure. Landing progress
shows simulation/wall elapsed time, per-UAV mode, armed state, altitude, and state
age. Cleanup requires settled ground and observed motor shutdown/disarm;
low altitude or a requested disable alone is insufficient.
Landing allows 60 simulation seconds, with separate 60-wall-second watchdogs
for a stalled clock or missing flight state (`scen1` allows 120 simulation seconds
for landing). A slow but progressing simulation can
therefore take longer than four wall minutes. Final mission and landing status
is preserved in `status/mission.json`.
Failures during `mpc` startup or its attached GUI use the same automatic cleanup.
The separate `gui` command leaves the stack running when its viewer exits.
Use `stop --force` only for a crashed simulation when normal landing cannot
complete. A failed command in an `&&` chain skips later cleanup commands.

Ownership is per UAV through token-checked `FlightCommand` services. Lost or
mismatched ownership stops publishing and triggers cleanup of remaining owned
UAVs; cleanup never commands released or unowned vehicles. New missions require
fresh connected flight state, settled ground and disabled motors.
A timed-out worker is disabled until the controller restarts; extending the
tracking duration cannot repair it. Preserve the failed run before restarting.

One flight node/process per UAV owns body `/uavN/cmd_vel` and `/uavN/enable`.
C++ `SoloFlightSafety` remains between these commands and native velocity
control: independent simulation/wall expiry triggers descent even after process
death, and motor disable occurs only after settled ground. Inspect actual
`/uavN/motor_speed` and `/uavN/safety_active`; never bypass safety by commanding
internal velocity-controller topics during recovery.

## Evidence and evaluation

Use the actual `runs/<run-id>/` directory, not a changing `runs/latest` link.

| Location | Evidence |
|---|---|
| `config/` | Fleet/scenario/controller snapshots, resolved `mpc-config.json`, world and bridge files |
| `metadata/` | Manifest, controller and upstream source hashes, dependency fingerprints |
| `status/` | Startup, live health, readiness, world-frame state and node initialization |
| `telemetry/mpc.jsonl` | Measured state, solver results, worker timing, controller decisions |
| `telemetry/truth.jsonl` | Independent Gazebo poses and target positions |
| `optimizer-inputs/<uav>/` | Bounded complete solver calls for numerical replay |
| `reports/evaluation.json` | Sampled geometry, tracking, compute, and simulation speed |
| `logs/`, `gazebo-home/` | ROS/process logs, native flight/safety state, renderer diagnostics |
| `recorded-mission.mp4` | Optional combined video |

The image ID is saved beside the run as `runs/<run-id>.image-id.json`.
Evaluate after landing, before stopping the container:

```bash
./scripts/dev evaluate --run-dir /workspace/runs/YOUR_RUN_ID
```

Evaluation uses measured Gazebo geometry and reports its source. The airborne
filter requires every UAV above the lower of `flight_altitude - 0.5` and
`0.9 * flight_altitude` (2.5 m for the 3 m presets); revisit it when changing altitude.
Inspect solver failures and measured motion alongside the summary. Sampled
clearance and successful commands alone do not establish continuous safety.

The three current presets completed recorded full-fleet native missions on
November 5, 2026: 30 tracking simulation seconds for the two 3 m presets and 90 seconds
for `scen1`. Default camera perception and per-UAV LiDAR mapping were enabled;
no ground-truth target override was used (detection has since been removed).
Telemetry confirmed settled landing,
released ownership and actual rotor shutdown for every vehicle. Finalized videos
were decoded and visually sampled with the original composition intact. See
[Development](DEVELOPMENT.md#native-migration-verification) for exact run
directories, solver failures, earlier failed attempts and bounded-run limitations.
No new CPU/performance acceptance thresholds apply.

## Recording

The Gazebo ground uses asphalt gray `#404341` with yellow `#C4B06A` double
centerlines and white `#C5C8C6` dashed lane dividers, edges and zebra crossings.
Display colors are converted to linear material values for Ogre2; illumination
and shadows still affect the captured pixels.
Eight-metre-wide streets repeat on a fixed 24 m world-aligned perpendicular
grid across the ground, independently of scenario obstacles and target routes.
Buildings and trees can cover the paint. Markings are batched visual meshes
6 mm above the ground, visible to cameras/GUI but excluded from LiDAR and
collision geometry. Restart the development stack to regenerate the scenery;
release images need rebuilding.

Select `--record-video` at launch; it also enables the UAV cameras shown in the FOV panels.
Normal `stop` and automatic shutdown after a failed `mpc` or `mission run`
finalize `recorded-mission.mp4`; successful mission completion and normal GUI
closure do not. Recording starts at takeoff and continues through landing.

MP4 recording streams composed BGR frames directly to FFmpeg, trying
`h264_nvenc` first. Actual initialization, pipe, flush or segment verification
failures switch permanently to CPU `libx264` for that recording. CPU encoding
and decoding use at most two codec threads, with one filter thread; existing
flight/optimizer limits are unchanged. NVIDIA containers need the `video` driver
capability from `compose.nvidia.yaml`. Release images require rebuilding.

No raw-frame files are written. Independently finalized one-second compressed
H.264 segments live under `recorded-mission.mp4.encoding/`. Fragmented MP4 also
allows recovery of completed keyframe fragments from an interrupted segment.
Only the current second of BGR input is retained in RAM (at most 20 frames,
about 124 MB at Full HD), allowing a failed GPU segment to be retried on CPU.
Segments use a common timebase and no B frames. Composition and source-time
sampling remain unchanged. Encoder stalls are bounded to ten seconds per frame
and flush to fifteen seconds; recording never controls flight commands.

Normal `./scripts/dev stop` lands first, then waits for recorder finalization
before stopping the container. Finalization remuxes decodable segment prefixes,
checks dimensions/FPS and decoded counts, and publishes `recorded-mission.mp4`
whenever usable footage exists. Healthy footage is joined without re-encoding.
Missing frames use held images at their original 20 FPS simulation-time slots;
only this recovery path re-encodes on CPU with the same thread limits. Leading
holes hold the first retained image. A recording with any such holes is
**partial**, even though the MP4 is playable. No usable frames, disk failures,
or failed output verification remain explicit failures. Cleanup still stops
the stack. The finalization wait budget is five minutes plus four times input
video duration. Individual segment decode checks allow sixty seconds.

Inspect `status/recording.json` and these telemetry files:

- `recorded-mission.timestamps.encoder.log`: encoder errors, fallbacks and summary.
- `recorded-mission.timestamps.inputs.jsonl`: original attempted frames and source stamps.
- `recorded-mission.timestamps.jsonl`: published output frame/source mapping;
  `held`, `source_frame` and `playback_ns` describe recovery precisely.
- `recorded-mission.timestamps.recovery.json`: retained/dropped/held counts,
  gap frame ranges (end exclusive), fallback events and partial/interrupted state.

Original source stamps are preserved on retained frames. Held frames carry the
retained image's source stamps and the missing slot's simulation stamp, so lost
frames and encoder restarts never compress simulation time. Compressed segments
and metadata remain available for audit; they contain no raw backups.

After a killed recorder or forced shutdown, first stop the stack, then recover
retained compressed footage using the simulation image (or a host with Python,
OpenCV and FFmpeg):

```bash
PYTHONPATH=ros_ws/src/solo_mpc python3 -m solo_mpc.core.video runs/RUN_ID --recover
```

Do not run recovery against a live recorder. Interrupted recovery is always
reported as partial: footage after the last journaled input cannot be inferred.
Keep the compressed segments, metadata and timestamp journals until inspected.
The command can be repeated and never needs uncompressed frame files. A machine
power loss can still lose unsynced data; recovery cannot restore absent inputs.
Direct Compose shutdown bypasses the normal handshake and retains its existing
two-minute recorder shutdown allowance. Installed releases can request normal
finalization with `python3 -m solo_mpc.core.video /workspace/runs/RUN_ID`
through the container entrypoint after landing and before stopping.

The 1920 × 1080 / 20 FPS output combines a target-following overhead view,
sensor-observed occupancy map, state instruments, and a FOV panel per UAV.
The state header labels total controller delivery time as `Fleet wall`, not
solver time. Each UAV row has a single `CPU solve time: … ms` timing line,
colored red for failed nominal solves even when a retained stop plan succeeds.
Stale or missing timing values display `--`. Detailed wall times, iterations,
mapping, planning, setup, worker, input preparation and capture costs remain
available in `telemetry/mpc.jsonl`.
The right-hand square panels shrink and stack vertically to fit every UAV;
the left map/state panels share the same displayed width and divide the height
to keep every state row visible for three- and five-UAV fleets. Compact state
rows reclaim the space from the removed timing lines; the map and state panels
automatically resize together at their shared width. Labels and fleet
colors follow the configured order.
Colored coverage rays/outlines appear in the overhead view and GUI, but are
excluded from UAV images and LiDAR. They show flat-ground camera projections;
obstacles can occlude the view. Distant UAVs can be outside the overhead frame.

State instruments use world ENU position (m), velocity (m/s), estimated and
commanded acceleration (m/s²), and measured yaw. Acceleration estimates are
smoothed velocity derivatives, excluding gravity. Commands precede publisher
guards. Arrow scales are 2 m/s or 2 m/s² at the rim. Missing/stale values show
`--`. Map cells show occupied/free/unknown sensor evidence, not obstacle truth.
The XYZ rows are position, velocity, estimated acceleration, and commanded
acceleration, with colors and units in the shared footer instead of a repeated
label column.

The state header shows `Fleet wall` (fleet cycle wall time, including input
preparation, mapping, planning, solving, coordination, and reference publishing),
and `Delay` (simulation time from the cycle's input timestamp to the displayed
video tick). Per-UAV solve CPU time is distinct from solver wall time
(`solver_ms` in telemetry); both include fallback inside the solve call.
`Nominal` counts successful nominal
solves out of the configured fleet; failures and the controller reason remain
visible. Status is sampled only after its `reported_stamp`, so completed solves
cannot appear on earlier frames. Reports older than one simulation second show
`STALE` and unavailable metrics; startup shows `WAITING`.

Feeds align by original simulation timestamps; slower feeds hold their last
frame. `telemetry/recorded-mission.timestamps.jsonl` maps output frames to time.
Every output frame advances 0.05 simulation seconds, including when physics
runs slower than wall time. FPS and playback speed need no slowdown multiplier.
Each combined-frame record also includes `source_stamps_ns` for the overhead,
UAV images, and occupancy map; repeated source stamps identify held feeds.
The Full HD overhead bridge and recorder use reliable ROS delivery with a
20-image history so lost DDS fragments can be retransmitted. Best-effort
delivery of these large images can cause periodic freezes even when Gazebo
renders at 20 FPS; the smaller UAV camera feeds retain sensor-data QoS.
Restart the stack after changing recording code or bridge configuration.
Rendering/encoding adds load. Add rosbag recording from `shell` only when needed,
using a fresh output directory and stopping it before the container.

## GPU and display

Rendering sensors need a working server renderer even without a GUI. Run
`./scripts/dev doctor` against a running stack; it reads the server's Ogre log.
An accelerated GUI does not prove server acceleration.

- WSL2: enable Docker Desktop's Ubuntu integration. The runner automatically
  adds `compose.wslg.yaml` when WSLg sockets exist.
- Native Intel/AMD: select `COMPOSE_FILE=compose.yaml:compose.gpu.yaml` and set
  `RENDER_GID`/`VIDEO_GID` from your actual `/dev/dri` devices.
- NVIDIA: select `COMPOSE_FILE=compose.yaml:compose.nvidia.yaml` with host drivers
  and NVIDIA Container Toolkit. Add `compose.wslg.yaml` or `compose.x11.yaml`
  when a viewer needs that display path.
- Software fallback: `LIBGL_ALWAYS_SOFTWARE=1 ./scripts/dev mpc`. Expect slower
  startup and lower wall-time sensor rates.

Explicit `COMPOSE_FILE` overrides automatic selection; use the same files for
start, inspection, and stop. Native X11 needs `DISPLAY`, `/tmp/.X11-unix`, and
local Docker X access (for example `xhost +local:`). DRM group IDs must match the
Docker daemon host. For example, discover them using
`stat -c %g /dev/dri/renderD128` and `stat -c %g /dev/dri/card0` with real paths.

## Diagnose failures

```bash
./scripts/dev status
./scripts/dev mission status
./scripts/dev doctor
./scripts/dev logs
```

Inside `./scripts/dev shell`:

```bash
ros2 topic echo /uav1/flight/state --once
ros2 topic echo /uav1/ground_truth/odometry --qos-reliability best_effort --once
ros2 topic echo /uav1/mpc/pose --qos-reliability best_effort --once
ros2 topic echo /uav1/motor_speed --qos-reliability best_effort --once
ros2 topic echo /uav1/safety_active --qos-reliability best_effort --once
ros2 topic hz /uav1/scan
ros2 topic echo /mpc/setpoint_status --once
timeout 5s gz topic -e -t /world/mpc_small50/stats
```

Use the configured world name. Exit 124 from `timeout` is expected. A stopped
clock suggests paused/stalled physics or bridge/rendering failure. A slow clock
with progressing odometry and motor messages means startup is advancing slowly.
Missing poses require checking `OdometryPublisher`, source stamps, bridges and
`frame_node`; missing motor/safety data requires native plugin and bridge checks.
Missing upstream sources require submodule initialization; worker solve timeouts
require fixing the cause and restarting.

`[ros2run]: Process exited with failure 1` reports the mission client's failure;
the preceding mission error and `status/mission-failure.json` (saved before
automatic cleanup) identify the reason. For other mission subcommands, inspect
`status/mission.json`. New failures also save `status/mission-trigger.json`
immediately before landing: it freezes the failing phase, controller results,
setpoint status and per-UAV flight state. The same record remains under
`failure_context` in mission status through subsequent cleanup requests.
The altitude-envelope error includes the affected UAVs, measured world ENU
altitudes, and minimum allowed altitude. A completed landing does not clear a
tracking failure. Check measured clearance, flight/safety errors, actual rotor
speeds and `obsolete_cycle` records before retrying. Faster pacing or heavy load
can let mapping/solver cycles exceed independent simulation-time command expiry.
Long SEARCH approaches depend on prediction speed, acceleration, lag and
stopping horizon; compute headroom alone does not establish native braking
response. Restart after changing settings. Keep expiry, ownership, motor-shutdown
confirmation and the altitude guard enabled.
