# Controller parity verification

[Development](DEVELOPMENT.md) · [Validation record](VALIDATION.md)

The supplied simulation uses three UAVs. Verify controller behavior through
simulation evidence and captured optimizer inputs. Numerical replay alone does
not establish successful flight or achieved control rates.

## Simulation verification

Do not write automated tests. Run a representative mission, evaluate after
landing, and stop normally to finalize the recording:

```bash
./scripts/dev mpc --record-video
./scripts/dev mission run --seconds 20
./scripts/dev evaluate
./scripts/dev stop
```

Inspect `telemetry/mpc.jsonl`, `telemetry/truth.jsonl`,
`reports/evaluation.json`, process logs, and `recorded-mission.mp4` under the
saved run directory. For serial/parallel or camera/ground-truth comparisons,
use separate runs with matching settings and preserve both run directories.
Review worker timing, failures, tracking, and landing alongside numerical output.
Controller provenance remains in
[UPSTREAM.json](../ros_ws/src/solo_mpc/UPSTREAM.json); the simulation needs no
external upstream checkout.

## Physical adaptations

The ROS controller uses measured FCU state, real scan projection, camera target
observations, and common-world frame alignment. Ground-truth target input is
available explicitly through `./scripts/dev mpc --ground-truth-target`; it
preserves measured positions while estimating velocity and confidence.

Workers exchange a frozen fleet prediction snapshot between cycles. Corridors
include LiDAR sampling, noise, and occupied-cell quantization margins. Retained
stopping plans advance with simulation time and require measured-state agreement;
failed nominal solves remain visible to mission supervision.

The three-UAV controller keeps its 0.2-second IPOPT CPU cap and one-second worker
phase deadline. The independent publisher samples trajectories at elapsed
simulation time and retains their source timestamps for expiry. Inspect live
worker timing and failure diagnostics to assess runtime deadline performance.

## Optimizer capture and replay

`diagnostic_capture_cycles` retains a bounded ring of optimizer calls per UAV
(default 128; 0 disables capture) under
`runs/<run-id>/optimizer-inputs/<uav>/solve-NNNN.json`. A pending record is written
before entering IPOPT. Records contain resolved settings, solver inputs, nominal
and backup warm starts, source hashes, timestamps, outcomes, and solved arrays.
Copy records before later calls overwrite them.

Replay a captured call in the simulation environment:

```bash
PYTHONPATH=ros_ws/src/solo_mpc OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 -m solo_mpc.replay_optimizer \
  runs/RUN_ID/optimizer-inputs/uav1/solve-0001.json
```

Replay uses recorded budgets. `--iteration-only` removes the CPU cap for a
separate numerical diagnostic. Timing and dependency differences can affect
results. Optimizer capture does not reconstruct sensor, map, or camera history.

For supervised simulation checks and evidence collection, follow
[Getting started](GETTING_STARTED.md) and [Operations](OPERATIONS.md#logs-and-evaluation).
