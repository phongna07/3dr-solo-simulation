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

For the kilometre-wide upstream field at 30 m flight altitude:

```bash
./scripts/dev mpc --preset scen1 --record-video
./scripts/dev mission run --seconds 90
./scripts/dev evaluate
./scripts/dev stop
```

Add `--ground-truth-target` to isolate flight/control from marker perception.
The complete route is much longer than this representative check and contains
13 m/s sprint legs. A duration-limited mission is not evidence of completing
the route or all handoffs. Retain the actual run ID and inspect the evaluation,
per-UAV solver/worker timing, camera observations, landing and finalized video.
Budget wall time from the selected pacing and observed clock rate, plus
takeoff/landing. Recording temporarily retains raw video until finalization;
check free disk space before long missions.

To watch distant followers rendezvous with a leader already over the target,
select `--preset large200_5uav_separated` in the same commands. Keep camera mode
(the default) to check that UAV 1 retains the pink target while the other four
approach from about 9–37 m away. UAV 3 starts southwest of the leader to reduce
crossing during joining. Its UAV prediction speed/acceleration bounds are
2.2 m/s and 3 m/s²; target speed is 0.6 m/s on its first two legs, then 1.0 m/s.
Allow enough simulation time for joining and subsequent tracking, and inspect
recorded modes and measured separations; loaded renderers extend wall time.

The original large preset's route takes about 464 simulation seconds, with
several early slalom bends through nearby obstacles: at least 39 wall minutes at
0.2×. Separated starts and slower first legs need their own duration budget;
target-loss pauses can extend either route. Consult the actual clock rate.
A time-limited run covers only part of it; rendering five sensors and a large
obstacle field can make simulation slower than wall time. Use the observed clock
rate when choosing a verification duration. Select `--preset small50_3uav` to
return to the original configuration; see [Configuration](CONFIGURATION.md#five-uavs-in-a-200--200-m-map)
for regeneration and parameter details.

| Command | Purpose |
|---|---|
| `mpc --ground-truth-target` | Exact timestamped target input; omit detectors/cameras unless recording |
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

All four presets completed recorded full-fleet native missions on November 5,
2026: 30 tracking simulation seconds for the three 3 m presets and 90 seconds
for `scen1`. Default camera perception and per-UAV LiDAR mapping were enabled;
no ground-truth target override was used. Telemetry confirmed settled landing,
released ownership and actual rotor shutdown for every vehicle. Finalized videos
were decoded and visually sampled with the original composition intact. See
[Development](DEVELOPMENT.md#native-migration-verification) for exact run
directories, solver failures, earlier failed attempts and bounded-run limitations.
No new CPU/performance acceptance thresholds apply.

## Recording

Select `--record-video` at launch, optionally with `--ground-truth-target`.
Normal `stop` and automatic shutdown after a failed `mpc` or `mission run`
finalize `recorded-mission.mp4`; successful mission completion and normal GUI
closure do not. Recording starts at takeoff and continues through landing.

MP4 encoding automatically tries FFmpeg `h264_nvenc` by encoding a frame at the
actual recording resolution. This checks the encoder, device and driver at
runtime; an NVIDIA GPU or an encoder listing alone is insufficient. If the
probe fails, recording uses CPU `mpeg4`. No encoder setting is required on
NVIDIA, Intel/AMD or software-rendered machines. NVIDIA containers need the
`video` driver capability, included in `compose.nvidia.yaml`. GPU/display setup
below still governs Gazebo rendering independently of video encoding. Rebuild
with `./scripts/dev build sim` for the FFmpeg dependency; release images must
also be rebuilt.

Both encoders receive the same composed BGR frames at the same FPS. Encoder
selection changes compression only. The recorder logs the selected encoder,
fallback reasons and verified frame count to the process log and
`telemetry/recorded-mission.timestamps.encoder.log`.
An encoder pipe accepting a frame does not guarantee that it reached the MP4:
the recorder keeps an exact, uncompressed frame journal in
`recorded-mission.mp4.encoding/` until finalization. An NVENC failure during
writing, flushing or output verification replays the entire journal on CPU,
including any partially submitted frame. Finalization decodes/counts frames
and checks resolution/FPS before publishing the MP4 and removing the journal.
The journal needs about 7.5 GB per simulation minute at 1080p/20 FPS, in addition
to the compressed video. Keep enough free space for the whole recording.
The recorder checkpoints and evicts written journal pages every 32 MiB to bound
dirty page-cache growth during long flights; all raw frames remain on disk until
verification, with the same recovery procedure. Only the recorder waits for these
disk checkpoints. Slow disks can still delay recording.

`./scripts/dev stop` requests recorder finalization after landing and waits for
`status/recording.json` to confirm a verified MP4 **before** stopping the container.
This keeps long recordings outside the launch/Compose shutdown deadline. The
wait budget is five minutes plus four times the recorded video duration; decoded
verification allows one minute plus twice the duration. Verification uses two
decoder threads. A verification timeout retains the encoded output instead of
triggering a full CPU replay. A genuine NVENC failure replays to a separate
`video-cpu.mp4`, preserving the first encode until successful verification.
Normal stop returns failure if recording cannot be finalized, while still
stopping the stack. Successful finalization removes the temporary journal.

Direct Compose shutdown and `stop --force` bypass this handshake and retain the
two-minute recorder shutdown allowance. For installed releases, explicitly run
`python3 -m solo_mpc.core.video /workspace/runs/latest` through the container
entrypoint after landing and before stopping (see the deployment guide).
CPU/disk errors, forced termination or exhausted finalization time are failures;
an unfinished video is not published as `recorded-mission.mp4`. Retained
`frames.bgr`, `input.json`, timestamps and encoder logs permit recovery without
discarding earlier frames. Do not delete them before recovery. For example,
for a journal whose `input.json` specifies 1920×1080 at 20 FPS, encode to a new
path with:

```bash
ffmpeg -f rawvideo -pixel_format bgr24 -video_size 1920x1080 -framerate 20 \
  -i runs/RUN_ID/recorded-mission.mp4.encoding/frames.bgr \
  -c:v mpeg4 -q:v 2 -pix_fmt yuv420p -movflags +faststart \
  runs/RUN_ID/recovered-mission.mp4
```

Use the journal's actual dimensions/FPS and compare the decoded frame count
(`ffprobe -v error -count_frames -show_streams VIDEO`) with the timestamp journal
before accepting a recovered file. Run recovery in the simulation image if the
host lacks FFmpeg. A disk-write failure can leave a partial trailing raw frame;
the error and journals must be inspected rather than claiming a complete video.

The 1920 × 1080 / 20 FPS output combines a target-following overhead view,
sensor-observed occupancy map, state instruments, and a FOV panel per UAV.
The right-hand square panels shrink and stack vertically to fit every UAV;
the left map/state panels share the same displayed width and divide the height
to keep every state row visible for three- and five-UAV fleets. Labels and fleet
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

The state header shows `Solve CPU max` (the largest per-UAV process CPU time
inside the unchanged upstream solve call, including fallback), `MPC` (fleet
cycle wall time, including input preparation, mapping, planning, solving,
coordination, and reference publishing), and `Delay` (simulation time from the
cycle's input timestamp to the displayed video tick). CPU time is distinct from
solver wall time (`solver_ms` in telemetry). `Nominal` counts successful nominal
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
renders at 20 FPS; the smaller perception feeds retain sensor-data QoS.
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
