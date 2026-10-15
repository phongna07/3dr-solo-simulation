# Reproducibility and dependency releases

[Back to the project overview](../README.md)

Use [compose.release.yaml](../compose.release.yaml) for experiments shared by
image digest. It runs installed application code from `/opt/solo_ws/install`,
with no source checkout mounted. Configuration is a separate read-only input;
only `runs/`, `artifacts/`, and temporary storage are writable. Preserve the run's
configuration snapshots as well as its platform and image digest.

[compose.yaml](../compose.yaml) remains the development workflow. It explicitly
sets `SOLO_DEV=1` to import live modules and launch files from the checkout. Its
image digest identifies the dependency environment, not the running application.

## Deploy a tested image without a checkout

Download the platform-specific runtime digest from the `release-images` workflow's
acceptance artifact or job summary. Do not substitute the input archive digest.
The workflow uses native AMD64 and ARM64 runners and publishes each platform only
after its acceptance test passes. A companion smoke pass validates ROS/MAVROS
loading; it does not validate hardware flight or FCU connectivity.

Use an empty directory, with Docker Compose and host Python 3 installed. Replace
the example image reference with the tested digest for the machine's architecture:

```bash
mkdir solo-release
cd solo-release
export IMAGE='ghcr.io/OWNER/solo-simulation@sha256:ACTUAL_PLATFORM_DIGEST'
docker pull "$IMAGE"
extract=$(docker create --entrypoint /bin/true "$IMAGE")
docker cp "$extract:/opt/solo/deploy/." .
docker cp "$extract:/workspace/config" config
docker rm "$extract"
cp .env.release.example .env
mkdir -p runs artifacts
# Edit .env: both runtime digests and RELEASE_PLATFORM=linux/amd64 or linux/arm64.
# Set LOCAL_UID / LOCAL_GID for the account owning runs/ and artifacts/.
./release pull
./release doctor
./release up sim
./release logs -f sim
```

The helper rejects tags and unsupported platforms before starting services. The
standalone Compose file also supports direct Docker Compose commands; use digest
references there too. Both image variables are required for Compose interpolation,
even if only one service is started. Configuration defaults to the copied `config/`.
To change controller settings or scenarios, copy the desired YAML under `config/`
and set `MPC_SETTINGS` / `MPC_SCENARIO` to its `/workspace/config/...` path.
Never reuse the development `.env`, which points those variables at checkout files.

The release root filesystem is read-only. `/tmp` is a private tmpfs for HOME,
Gazebo caches, and numerical JIT caches. Bind directories must already exist and
be writable by the configured UID/GID; Compose never creates them as root.
Start the mission explicitly after readiness:

```bash
./release doctor  # Reports the live Gazebo server renderer after startup.
./release exec -T sim /usr/local/bin/solo-entrypoint ros2 run solo_mpc mission run --seconds 20
./release exec -T sim /usr/local/bin/solo-entrypoint ros2 run solo_mpc mission stop
./release stop sim
```

For GPUs/serial devices, use the matching Compose override with the release base
and the actual device group IDs. See [WSL and GPU](WSL_GPU.md). `release` itself
uses the headless standalone file; custom override deployments should invoke
`docker compose --env-file .env -f compose.release.yaml -f ...` consistently.

## Build and runtime separation

Compiler stages build MAVROS, application interfaces, SITL and Gazebo plugins.
Final images copy normal ROS installs, SITL binaries/default parameters, plugin
libraries, models/worlds, GeographicLib data, numerical runtime dependencies, and
license/notice files. They contain no ArduPilot/MAVROS/application build trees,
Python 3.11 build environment, uv, or source checkout.

ROS Debian dependencies include development files even for runtime packages.
The final filesystem removes toolchain executables, headers and static/object
libraries, retaining shared libraries, ROS package resources and dependency
copyright files. `/opt/solo/removed-build-files.lock` records those removals;
`apt-packages.lock` records package versions before this pruning. Runtime images
are flattened so deleted build material does not remain in lower image layers.
APT modification of a pruned release image is unsupported; rebuild a new release.

## Locked and preserved inputs

| Input | Policy / location |
|---|---|
| ROS base and uv images | Digest pins in Dockerfile and `locks/versions.json` |
| ArduPilot, MAVROS, Gazebo plugin | Exact commits; ArduPilot submodule manifest |
| Python dependencies | Hash-locked `locks/*lock`; separate build/runtime environments |
| GeographicLib models | Payload hashes verified against `locks/geographiclib.sha256` |
| APT packages | Downloaded `.deb` files, SHA256SUMS, package indexes and repository metadata retained in the input image |
| Source, flags, environments | Source snapshot, upstream Git trees, build arguments, downloaded Python cache and installed build environments retained in the input image |

APT discovery still uses mutable repositories. A later fresh build may resolve
new patches; it is a new candidate requiring validation. Runtime stages use only
the exact archived `.deb` set from their compiler stages, with no live APT
repository access. Changed dataset payloads fail the build instead of silently
changing the release. Data, base filesystem and Python environments are also
preserved in the matching input image.

The release workflow saves checksummed runtime/input archives, tests on a separate
fresh native runner with **no checkout**, then pushes separate runtime/input
images plus the clean base to GHCR. Acceptance evidence records each platform
and the runtime, input and clean-base registry digests. CI artifacts expire after
90 days; preserve the digest-addressed input/base images and acceptance evidence in your team's long-term archive. Do not delete
those packages while the release is supported.

Local equivalents:

```bash
docker build --target simulation_mpc -t solo-simulation:candidate -f docker/Dockerfile .
docker build --target simulation_inputs -t solo-simulation-inputs:candidate -f docker/Dockerfile .
./scripts/release-smoke solo-simulation:candidate sim artifacts/release-smoke
# For companion: targets companion / companion_inputs; smoke service companion.
docker build --target base_inputs -t solo-base:inputs -f docker/Dockerfile .
docker save solo-simulation:candidate solo-simulation-inputs:candidate solo-base:inputs -o release.tar
sha256sum release.tar
```

To rebuild the runtime/application using a preserved input image, override the
compiler stage contexts. This skips mutable downloads and can run without network:

```bash
docker pull ghcr.io/OWNER/solo-simulation-inputs@sha256:INPUT_DIGEST
inputs='docker-image://ghcr.io/OWNER/solo-simulation-inputs@sha256:INPUT_DIGEST'
# Use the matching archived source snapshot as the build context, with Dockerfile.
docker build --network=none \
  --build-arg ROS_BASE=ghcr.io/OWNER/solo-simulation-base-inputs@sha256:BASE_DIGEST \
  --build-context common_build="$inputs" \
  --build-context simulation_dependencies="$inputs" \
  --build-context simulation_mpc_dependencies="$inputs" \
  --target simulation_mpc -t solo-simulation:replayed -f docker/Dockerfile .
```

This reuses the preserved upstream compiler outputs; it is not a promise of
bit-for-bit recompilation of every upstream component. For exact deployment,
pull the tested runtime digest or load its checksummed image archive. The clean
base is preserved separately so replay does not depend on continued availability
of the original base registry; for local archives use `ROS_BASE=solo-base:inputs`.

## Validate and diagnose

`release-smoke` extracts all deployment files from the candidate image into a
temporary directory, checks installed imports and MAVROS overlay resolution,
verifies application/dataset hashes and absence of build environments, and checks
shared-library resolution. Simulation then requires three FCU connections, fresh
poses, all node constructors, and the dedicated Gazebo server renderer log.
It never arms vehicles. Logs and manifests remain in the requested evidence directory.

`doctor` reports host architecture and compares Docker daemon, requested platform
and image architecture; checks container writes as the configured user; reports device ownership/groups
and actual access; and reads only the per-run server Ogre log. No server log means
**unvalidated**, never assumed GPU acceleration. A software renderer is reported
explicitly. Run `./scripts/dev doctor --service companion` for the hardware profile.

Development runs still record live source hashes. Release runs copy
`application.sha256` from the image alongside configuration snapshots, package
locks and dataset checksums. `./scripts/dev lock` exports manifests to `artifacts/`;
record a registry digest separately when sharing an image.

Dependency updates require regenerating Python hashes, rebuilding the candidate,
and repeating native acceptance. ARM64 passes must be obtained from the native
workflow or target hardware; AMD64 emulation/build success is not ARM64 runtime
validation. Follow [hardware migration](HARDWARE.md) before physical flight.
