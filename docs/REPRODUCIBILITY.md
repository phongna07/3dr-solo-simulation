# Reproducibility and dependency releases

[Back to the project overview](../README.md)

Share a tested image by registry digest or image archive to run the same binary
environment across machines. The Dockerfile pins important inputs, but rebuilding
it later can still install different APT patches. The live repository is
bind-mounted, so reproducing an experiment also requires the same source and
configuration, including any uncommitted changes.

## Locked and recorded inputs

| Input | Policy / location |
|---|---|
| ROS base and uv images | Digest pins in the [Dockerfile](../docker/Dockerfile) and [versions.json](../locks/versions.json) |
| ArduPilot | `Copter-4.5.3`, exact commit and its submodules |
| Gazebo plugin | Exact source commit |
| MAVROS | 2.15.1, source commit `22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e` |
| ArduPilot build Python | Separate Python 3.11.16 environment |
| Waf Python dependencies | Hash-locked [ardupilot-py311.lock](../locks/ardupilot-py311.lock) |
| Python build backend | Hash-locked [build-backend.lock](../locks/build-backend.lock) |
| Simulation MPC dependencies | Hash-locked [mpc-py312.lock](../locks/mpc-py312.lock), Python 3.12 |
| APT packages and GeographicLib data | Installed versions/checksum recorded after build |

Gazebo is Harmonic / gz-sim8. Its APT patch version, compiler packages, and other
APT dependencies are not fixed by a snapshot mirror. A pinned base image does
not make the package repositories immutable. Long-term source rebuilds require
preserving matching `.deb` files or repository snapshots, Python artifacts and
runtime downloads, GeographicLib data, architecture, and build flags.

APT MAVROS packages supply dependencies, then the source overlay supplies the
pinned runtime. The APT manifest can therefore show a different MAVROS version.
Verify `ros2 pkg prefix mavros` resolves to `/opt/mavros_ws/install/mavros`, and
check that prefix's `share/mavros/package.xml`. The entrypoint sources ROS first,
then MAVROS, then the project overlay.

## Export the environment

After starting a simulation at least once:

```bash
./scripts/dev lock
```

This exports image manifests and image inspection data to `artifacts/`.
Simulation runs also record source/configuration hashes, package/submodule locks,
and the image ID. [Operations](OPERATIONS.md#logs-and-evaluation) describes these
outputs. A local image ID identifies a local image but does not provide a registry
address another machine can pull.

For a comparable experiment, preserve the image identity, repository revision
and modified source, run input snapshots, renderer/platform details, and measured
logs. Controller restarts have their own hashes in `controller_sources.jsonl`.
Do not rely only on the launch-time Git commit when the working tree changed.

## Share a tested image

Replace the example registry with one your team uses:

```bash
docker tag solo-simulation:gd0 registry.example.org/solo/simulation:gd0-v1
docker push registry.example.org/solo/simulation:gd0-v1
docker buildx imagetools inspect registry.example.org/solo/simulation:gd0-v1
```

Record the returned digest in each member's `.env`:

```dotenv
SIM_IMAGE=registry.example.org/solo/simulation@sha256:<actual-digest>
```

Pull that image with `docker compose pull sim`, then use the normal mission
workflow without rebuilding it. `./scripts/dev pull` pulls both configured
simulation and companion services, including the hardware profile; use the
service-specific Compose command if only the simulation image is published.
Registry publication is a separate release step, not part of `scripts/dev`.

Without a registry, create and transfer an archive:

```bash
docker save solo-simulation:gd0 -o solo-simulation-gd0.tar
sha256sum solo-simulation-gd0.tar
# On the receiving machine, verify the archive checksum, then:
docker load -i solo-simulation-gd0.tar
```

Keep large images, build/install directories, EEPROM files, and recordings out of
Git. Share source through Git and experiment evidence separately.

## Update dependencies

Edit the input requirements, regenerate using the pinned uv version, review
versions and hashes, then rebuild and validate a new environment release:

```bash
UV_CACHE_DIR=/tmp/solo-uv-cache uv pip compile locks/ardupilot-py311.in \
  --python-version 3.11 --generate-hashes --output-file locks/ardupilot-py311.lock

UV_CACHE_DIR=/tmp/solo-uv-cache uv pip compile locks/mpc-requirements.txt \
  --python-version 3.12 --generate-hashes --output-file locks/mpc-py312.lock
```

The Waf environment installs locked setuptools/wheel first, then the main lock
with `--no-build-isolation`. If either backend version changes, update
`build-backend.lock` consistently so source packages do not fetch an untracked
backend. Keep Waf's Python 3.11 dependencies separate from ROS's system Python
3.12 and the simulation MPC environment.

For ARM64 companion releases, validate package availability and binaries on the
Pi, install the required numerical dependencies explicitly, and export a separate
manifest. An amd64 APT lock cannot simply be applied to ARM64. For a multi-platform
registry image, record both the digest and the platform used. Follow the
[hardware migration stages](HARDWARE.md) before treating an image as flight-ready.
