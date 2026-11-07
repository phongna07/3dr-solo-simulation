# syntax=docker/dockerfile:1
ARG ROS_BASE=ros:jazzy-ros-base-noble@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca
FROM ghcr.io/astral-sh/uv:0.12.17@sha256:10787c682e4184e4f290de1171fd4703dc63de99221f10fe1c99002ce7fa9acc AS uv
FROM ${ROS_BASE} AS base_inputs
FROM base_inputs AS common_build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
COPY docker/preserve-apt.sh /usr/local/bin/preserve-apt
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-downloads
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-eigen3-cmake-module \
      ros-jazzy-ament-cmake ros-jazzy-rosidl-default-generators \
      ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-actuator-msgs ros-jazzy-nav-msgs ros-jazzy-sensor-msgs ros-jazzy-trajectory-msgs \
      ros-jazzy-std-srvs ros-jazzy-tf2-ros \
      python3-yaml python3-venv python3-colcon-common-extensions \
      curl ca-certificates git iproute2 procps \
      build-essential cmake pkg-config python3-dev libgmp-dev libeigen3-dev && \
    preserve-apt
ENV HOME=/tmp/solo-home RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONUNBUFFERED=1
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/solo-entrypoint"]

# Keep application changes after expensive dependency/compiler layers.
FROM common_build AS application_build
COPY external/motion_planning_MPC /opt/solo_ws/external/motion_planning_MPC
COPY src /opt/solo_ws/src
RUN source /opt/ros/jazzy/setup.bash && cd /opt/solo_ws && \
    colcon build --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF
ARG SOURCE_REVISION=unknown
RUN mkdir -p /opt/solo/build-inputs && \
    printf '%s\n' "$SOURCE_REVISION" > /opt/solo/build-inputs/source-revision
# Keep this workspace and its build inputs in a separate archival target.
COPY config /workspace/config
COPY scripts /workspace/scripts
COPY compose.release.yaml .env.release.example scripts/release scripts/doctor.py /opt/solo/deploy/
COPY locks /opt/solo/locks
COPY locks/versions.json /opt/solo/versions.json
COPY docker/entrypoint.sh /usr/local/bin/solo-entrypoint

FROM common_build AS simulation_dependencies
ARG BUILD_JOBS=4
COPY --from=uv /uv /usr/local/bin/uv
ENV UV_CACHE_DIR=/opt/solo/build-inputs/uv-cache
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic libgz-sim8-plugins libgz-sim8-dev libgz-gui8-dev \
      libgz-rendering8-dev libgz-plugin2-dev qtbase5-dev qtdeclarative5-dev cppzmq-dev \
      gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
      libegl1-mesa-dev libgl1-mesa-dri mesa-utils libqt5svg5 && \
    preserve-apt
RUN dpkg-query -W -f='${binary:Package}=${Version}\n' | sort > /opt/solo/apt-packages.lock && \
    printf 'BUILD_JOBS=%s\nGZ_VERSION=harmonic\n' "$BUILD_JOBS" \
      > /opt/solo/build-inputs/simulation-build-args
FROM simulation_dependencies AS simulation_mpc_dependencies
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz-bridge python3-opencv && preserve-apt
COPY locks/mpc-requirements.txt /opt/solo/mpc-requirements.txt
COPY locks/mpc-py312.lock /opt/solo/mpc-py312.lock
RUN python3 -m venv --system-site-packages /opt/venvs/mpc && \
    uv pip install --python /opt/venvs/mpc/bin/python --require-hashes -r /opt/solo/mpc-py312.lock
ENV PYTHONPATH=/opt/venvs/mpc/lib/python3.12/site-packages \
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 NUMBA_NUM_THREADS=1
# Keep encoder tools after the pinned numerical environment so adding them does
# not invalidate the solver dependency layers. Preserve debs for offline release.
RUN apt-get update && apt-get install -y --no-install-recommends ffmpeg && preserve-apt
FROM simulation_dependencies AS simulation_camera_build
COPY src/solo_mpc/gazebo /opt/solo_gazebo/src
RUN cmake -S /opt/solo_gazebo/src -B /opt/solo_gazebo/build \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/solo_gazebo && \
    cmake --build /opt/solo_gazebo/build -j"$BUILD_JOBS" && \
    cmake --install /opt/solo_gazebo/build
# Source, MPC submodule, Python downloads, APT debs/indexes and build flags
# are retained here for archival, never copied wholesale into runtime images.
FROM simulation_mpc_dependencies AS simulation_inputs
COPY --from=simulation_camera_build /opt/solo_gazebo /opt/solo_gazebo
COPY --from=application_build /opt/solo/build-inputs/source-revision /opt/solo/build-inputs/source-revision
COPY --from=application_build /opt/solo_ws /opt/solo_ws
COPY --from=application_build /workspace /workspace
COPY --from=application_build /opt/solo/locks /opt/solo/locks
COPY --from=application_build /opt/solo/versions.json /opt/solo/versions.json
COPY --from=application_build /opt/solo/deploy /opt/solo/deploy
COPY --from=application_build /usr/local/bin/solo-entrypoint /usr/local/bin/solo-entrypoint
COPY . /opt/solo/source-checkout

FROM base_inputs AS runtime_common
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive HOME=/tmp/solo-home \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
# Install from the exact debs used by the compiler stage, with no live APT access.
COPY docker/install-runtime.sh /usr/local/bin/install-runtime
RUN --network=none --mount=from=common_build,source=/opt/solo/build-inputs/apt,target=/inputs,ro \
    install-runtime ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cv-bridge ros-jazzy-image-transport \
      ros-jazzy-rosbag2-storage-mcap ros-jazzy-actuator-msgs ros-jazzy-nav-msgs ros-jazzy-sensor-msgs \
      ros-jazzy-trajectory-msgs ros-jazzy-std-srvs ros-jazzy-tf2-ros python3-yaml iproute2 procps
COPY --from=application_build /opt/solo_ws/install /opt/solo_ws/install
COPY --from=application_build /workspace /workspace
COPY --from=application_build /opt/solo/locks /opt/solo/locks
COPY --from=application_build /opt/solo/versions.json /opt/solo/versions.json
COPY --from=application_build /opt/solo/deploy /opt/solo/deploy
COPY --from=application_build /usr/local/bin/solo-entrypoint /usr/local/bin/solo-entrypoint
COPY docker/prune-runtime.sh /usr/local/bin/prune-runtime
RUN mkdir -p /workspace/runs /workspace/artifacts && \
    chmod 755 /usr/local/bin/solo-entrypoint

FROM runtime_common AS simulation_root
RUN --network=none --mount=from=simulation_mpc_dependencies,source=/opt/solo/build-inputs/apt,target=/inputs,ro \
    install-runtime gz-harmonic libgz-sim8-plugins ros-jazzy-ros-gz-bridge python3-opencv ffmpeg \
      gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
      libgl1-mesa-dri libegl1 mesa-utils libqt5svg5
COPY --from=simulation_mpc_dependencies /opt/venvs/mpc /opt/venvs/mpc
COPY --from=simulation_camera_build /opt/solo_gazebo/lib /opt/solo_gazebo/lib
RUN strip --strip-unneeded /opt/solo_gazebo/lib/*.so && prune-runtime

# Flatten the prepared filesystem so removed toolchains occupy no layers.
FROM scratch AS simulation_mpc
COPY --from=simulation_root / /
ENV HOME=/tmp/solo-home RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 \
    ROS_DISTRO=jazzy PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    PYTHONPATH=/opt/venvs/mpc/lib/python3.12/site-packages \
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 NUMBA_NUM_THREADS=1 NUMBA_CACHE_DIR=/tmp/numba-cache
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/solo-entrypoint"]
CMD ["ros2", "launch", "solo_mpc", "mpc.launch.py"]
