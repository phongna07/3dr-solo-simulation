# Deployment

[Overview](../README.md) · [Simulation](SIMULATION.md) · [Development](DEVELOPMENT.md)

Development uses live sources at `/workspace` with `SOLO_DEV=1`. Release mode
uses installed code under `/opt/solo_ws/install`, including the unchanged pinned
upstream solver/dynamics/license. It needs neither a source checkout nor Git at
runtime. Preserve platform, image identity, source hashes, and input snapshots.

## Build and preserve images

Initialize the submodule before building. Docker contains expensive dependency
layers separately from application/native camera layers. Numerical dependencies
are hash-locked in `locks/`; do not install the upstream plotting requirements.

```bash
git submodule update --init --recursive
./scripts/dev build sim
```

For an archive of a validated candidate:

```bash
docker build --target simulation_mpc -t solo-simulation:candidate -f docker/Dockerfile .
docker build --target simulation_inputs -t solo-simulation-inputs:candidate -f docker/Dockerfile .
docker build --target base_inputs -t solo-base:inputs -f docker/Dockerfile .
docker save solo-simulation:candidate solo-simulation-inputs:candidate solo-base:inputs -o release.tar
sha256sum release.tar
```

The runtime excludes compilers, build trees, source workspaces, Python build
environments, and dependency caches. Input images preserve the corresponding
sources, compiler outputs, APT payloads/indexes, Python locks/cache, datasets,
build flags, and upstream pins/licenses. Runtime APT installs use archived debs;
new dependency discovery can resolve different patches and needs fresh validation.
Updating APT inside a pruned runtime is unsupported; rebuild it.

`./scripts/dev lock` exports available manifests/image identity to `artifacts/`.
The existing `scripts/release-smoke IMAGE sim|companion EVIDENCE_DIR` helper checks
installed imports, hashes, library availability, filesystem restrictions, and
simulation readiness without arming. A flight mission remains separate evidence.
Validate each target platform natively before publishing; emulation does not
establish ARM64 runtime performance.

## Run without a checkout

Obtain a validated, platform-specific runtime digest. In an empty directory:

```bash
export IMAGE='ghcr.io/OWNER/solo-simulation@sha256:ACTUAL_PLATFORM_DIGEST'
docker pull "$IMAGE"
extract=$(docker create --entrypoint /bin/true "$IMAGE")
docker cp "$extract:/opt/solo/deploy/." .
docker cp "$extract:/workspace/config" config
docker rm "$extract"
cp .env.release.example .env
mkdir -p runs artifacts
```

Set both `SIM_IMAGE` and `COMPANION_IMAGE` to tested registry digests, even when
starting only one service. Set `RELEASE_PLATFORM` to the image platform and
`LOCAL_UID`/`LOCAL_GID` to the output-directory owner. Keep release configuration
paths or place custom YAML under `config/`; do not reuse development paths into
a checkout. The helper rejects tags and unsupported platforms.

```bash
./release pull
./release doctor
./release up sim
./release logs -f sim
```

After readiness, explicitly fly, evaluate, land, and stop:

```bash
./release exec -T sim /usr/local/bin/solo-entrypoint ros2 run solo_mpc mission run --seconds 30
./release exec -T sim /usr/local/bin/solo-entrypoint ros2 run solo_mpc evaluate
./release exec -T sim /usr/local/bin/solo-entrypoint ros2 run solo_mpc mission stop
./release stop sim
```

Release `stop` calls Compose directly; it does not request LAND. Confirm mission
cleanup first. Root/configuration filesystems are read-only; `/tmp` is tmpfs and
output mounts must already exist and be writable. For GPU/display/serial changes,
use `docker compose --env-file .env -f compose.release.yaml -f OVERRIDE ...`
consistently. The helper selects the standalone headless base only.

## Replay archived build inputs

Use the matching archived source snapshot and Dockerfile as context:

```bash
inputs='docker-image://ghcr.io/OWNER/solo-simulation-inputs@sha256:INPUT_DIGEST'
docker build --network=none   --build-arg ROS_BASE=ghcr.io/OWNER/solo-simulation-base-inputs@sha256:BASE_DIGEST   --build-context common_build="$inputs"   --build-context simulation_dependencies="$inputs"   --build-context simulation_mpc_dependencies="$inputs"   --target simulation_mpc -t solo-simulation:replayed -f docker/Dockerfile .
```

This reuses preserved compiler outputs; it does not promise bit-identical
upstream recompilation. For exact deployment, pull the tested runtime digest or
load its checksummed archive. Record runtime/input/base digests with the native
validation evidence and retain them for the supported release lifetime.

## Companion profile

The optional `companion` service remains a MAVROS/readiness foundation. It does
not run MPC, start sensor drivers, arm, or send LAND on shutdown. Its image
excludes Gazebo, SITL, and MPC numerical dependencies. Physical Pi/Solo flight
has not been validated; simulation mission commands reject hardware runs.

Configure [hardware.yaml](../config/hardware.yaml) only after verifying the real
firmware, SYSID, link routing, pinout, voltage, serial device, and baud. The
supplied `/dev/solo-fcu:921600` is a placeholder. Keep LD19/GoPro disabled until
explicit driver, calibration, and timestamp integration is implemented.

```bash
./scripts/dev build companion
export COMPOSE_FILE=compose.yaml:compose.hardware-serial.yaml
export FCU_DEVICE=/dev/solo-fcu
export DIALOUT_GID=$(stat -Lc %g "$FCU_DEVICE")
./scripts/dev companion
./scripts/dev doctor --service companion
./scripts/dev companion-stop
```

Use an actual stable host device and its group. The profile uses host networking;
readiness needs local pose as well as heartbeat. Logs are under `runs/hardware/`.
Connecting a Pi to laptop SITL additionally needs reachable MAVLink ports, DDS
routing, explicit control ownership, and clock/frame adapters; the default
simulation keeps these links inside one container. Physical control requires
separate localization, sensor, timing, failsafe, pilot-override, and vehicle-response
validation before any flight integration.
