# Simulation startup performance

`./scripts/dev mpc --record-overhead --record-cameras` starts one container,
then starts Gazebo, three SITL flight controllers, three MAVROS instances,
readiness, sensor bridges, MPC adapters, perception, and recorders concurrently.
The Docker `Started` message only describes the container process. It does not
mean the simulated vehicles have initialized their estimators.

## What the supplied trace shows

| Wall time | Completed | Remaining |
| --- | --- | --- |
| About 2 s | Most ROS constructors and recorders | Controller, MAVROS configuration, FCUs |
| About 5 s | Controller and all 22 configured MAVROS plugins per UAV | FCU connection and telemetry |
| About 14 s | All three FCU connections | Telemetry acknowledgements and local pose |
| About 17 s | Position/attitude telemetry requests accepted | First EKF local poses |
| 32 s and later | All of the above | Local position estimate on every UAV |

The three UAVs wait in parallel. The dominant delay is flight-controller
initialization under slow simulation, rather than a sequential service startup.
MAVROS plugin bootstrap has to use ROS parameter services because the pinned
MAVROS plugin nodes ignore global command-line parameter files. That work is
complete early in this trace; removing its readiness check would not resolve
the long local-pose wait.

An accepted MAVLink message-interval request means the FCU accepted a requested
stream rate. ArduPilot still omits `LOCAL_POSITION_NED` until its estimator can
provide position and velocity. In the pinned firmware,
[`send_local_position()`](https://github.com/ArduPilot/ardupilot/blob/ee14ff3822161368ec403700bf96d02b84def06c/libraries/GCS_MAVLink/GCS_Common.cpp#L2765)
returns without sending when either estimate is unavailable. Barometer startup,
EKF tilt/yaw alignment, GPS detection, origin selection, and gyro-bias convergence
all happen before usable local poses. The estimator also requires ten seconds
of passing GPS checks in simulation time; see
[`calcGpsGoodToAlign()`](https://github.com/ArduPilot/ardupilot/blob/ee14ff3822161368ec403700bf96d02b84def06c/libraries/AP_NavEKF3/AP_NavEKF3_VehicleStatus.cpp).

## Rendering magnifies the estimator wait

On the assessed WSL host, the Gazebo server's Ogre log identified
`llvmpipe (LLVM 20.1.2, 256 bits)`: CPU software rendering. Host GPU devices and
a working WSLg display did not give this container's headless server hardware
acceleration. The baseline advanced only 7.4 simulation seconds in about 66
wall seconds, and 35.3 simulation seconds in about 308 wall seconds. A timeout
of 600 seconds is a deadline, not an intentional startup sleep.

The flight controllers use Gazebo's lockstep simulation clock. Slow physics or
sensor rendering therefore stretches their initialization in wall time. The
relationship is approximately:

```text
wall time = simulation time / real-time factor + process startup overhead
```

The overhead option adds a 1920×1080 camera at 20 frames per simulation second:
41.5 million pixels, or about 124 MB of uncompressed RGB data, per simulation
second before transport copies. The three downward perception cameras together
produce about 0.61 million pixels per simulation second at 160×160 and 8 Hz.
LiDAR also needs rendering, so pixel counts alone do not predict total runtime.

`--record-cameras` reuses downward cameras that already run for perception.
Before this change, `--record-overhead` rendered and bridged the Full HD view
from launch, despite recording starting only after takeoff. The recorder returns
before image conversion and video encoding while grounded, so the preflight
overhead cost was rendering and transport rather than AVI disk writes.

## Changes

* The overhead bridge is lazy, and the overhead recorder creates its image
  subscription on the first connected, armed FCU state. Gazebo can skip unused
  rendering while disarmed. Activation happens before the takeoff recording
  gate; resolution, frame rate, target tracking, and video timestamps are retained.
  Gazebo's [Sensors system](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/sensors/Sensors.cc)
  skips rendering sensors with no transport connections. An external subscriber
  to the overhead image can still activate rendering before arming.
* Parameter bootstrap dispatches its next service phase as soon as the previous
  response is processed, eliminating empty 0.5-second polling intervals.
* Successful telemetry acknowledgements dispatch the next request immediately.
  Requests remain serialized per FCU, since both use MAVLink command 511;
  failed requests retain retry backoff.
* Startup progress reports recent real-time factor, elapsed simulation time,
  missing/stalled clock updates, and the latest FCU status. Changing clock values
  do not cause one report per second: transitions and the 15-second refresh remain.
* `health.json` and `ready.json` retain `first_ready_elapsed_wall_s`, so later
  status updates do not overwrite the time the fleet first became ready,
  measured from readiness monitor construction. The CLI prints its own total
  host wait duration, which also includes discovery and process initialization.

Readiness still requires every configured plugin, connected FCUs, acknowledged
telemetry, and at least three distinct, finite, fresh local poses, plus process
initialization for `mpc`. Hardware configuration and estimator checks are retained.

## Measured result

One fresh run before and one after the change on 15 October 2026 used the same
image, three-UAV scenario, WSL host (Core Ultra 5 225H, 14 CPUs available to
Docker), software renderer, and both recording flags. Neither run armed or flew.

| Measurement | Before | After |
| --- | ---: | ---: |
| Host readiness wait | 361 s | 234.4 s |
| Sustained simulation speed while disarmed | About 0.11× | About 0.18× |
| EKF initialization, from launch inventory start | 36.2 s | 24.9 s |
| GPS detection, from launch inventory start | 81.9 s | 53.8 s |
| EKF origin, from launch inventory start | 170.5 s | 111.6 s |
| Last blocking stage | First fresh local poses | First fresh local poses |

The host wait fell by about **127 seconds (35%)**. Both runs needed approximately
41 simulation seconds to supply the required poses. This is a paired observation,
not a startup-time guarantee; CPU contention, renderer, shader caches, and scene
complexity affect results. The controller finished constructing in about 3.6–3.7
seconds in both runs. Most of the saving comes from rendering less while waiting
for the same estimator initialization.

Local evidence is saved in `artifacts/startup-baseline.log`,
`artifacts/startup-improved.log`, matching `startup-*-container.log` files, and
health/startup/manifest snapshots. The baseline run is
`runs/20261015T061044Z-20121`; the updated run is
`runs/20261015T061726Z-20498`. `startup-improved-overhead-topic.txt` shows an image
publisher with **no transport subscribers** during disarmed startup. These
artifacts and run directories are ignored by Git.

Validation: all **142 tests passed** in the simulation image (37 seconds),
including a real Gazebo rendering test. That test checks that the disarmed
overhead topic has no transport subscribers, activates it through armed
telemetry, moves the target, and decodes the finalized 1920×1080, 20 FPS video
with increasing simulation timestamps. ROS tests cover recorder activation,
telemetry serialization/retry, clock reset/stall reporting, and readiness gates.
The rendered-video test uses an isolated static scene and synthetic flight
telemetry; the startup benchmarks use all three live, disarmed FCUs.

## Measure and diagnose

Run the command and preserve its terminal report:

```bash
./scripts/dev mpc --record-overhead --record-cameras | tee artifacts/startup.log
./scripts/dev status
```

Inside `./scripts/dev shell`, inspect the server renderer and Gazebo statistics:

```bash
grep -E 'GL_RENDERER|GL_VENDOR' /workspace/runs/latest/gazebo-home/.gz/rendering/ogre2.log
timeout 5s gz topic -e -t /world/mpc_small50/stats
```

The readiness real-time factor uses a recent window of roughly ten wall seconds.
Gazebo's individual statistics samples can be bursty under lockstep rendering;
compare changes in `sim_time` and `real_time` over an interval. No clock progress
suggests paused/stalled physics, a failed bridge, or a rendering problem. A
progressing but slow clock with EKF/GPS messages indicates slow initialization.
A healthy simulation clock with a prolonged pose wait needs investigation of
the FCU estimator and telemetry rather than a larger timeout.

Use the [GPU guide](WSL_GPU.md) to configure host-specific acceleration and verify
the server renderer. Deferring overhead work improves disarmed startup; the Full
HD workload returns when armed and can still limit flight speed. Closing the GUI
does not remove camera/LiDAR rendering. A live overhead-topic viewer also removes
the preflight rendering saving.
