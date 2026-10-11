# syntax=docker/dockerfile:1
ARG ROS_BASE=ros:jazzy-ros-base-noble@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca
FROM ghcr.io/astral-sh/uv:0.12.17@sha256:10787c682e4184e4f290de1171fd4703dc63de99221f10fe1c99002ce7fa9acc AS uv
FROM ${ROS_BASE} AS common
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
# APT provides build/runtime dependencies; the pinned source overlay below is MAVROS 2.15.1.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs \
      ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-eigen3-cmake-module \
      ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-rosbag2-storage-mcap \
      python3-yaml python3-venv python3-pytest python3-colcon-common-extensions \
      geographiclib-tools curl ca-certificates git iproute2 procps tmux \
      build-essential cmake pkg-config python3-dev libgmp-dev \
      libasio-dev libgeographiclib-dev libconsole-bridge-dev libeigen3-dev && \
    /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh && \
    test -f /usr/share/GeographicLib/geoids/egm96-5.pgm && \
    rm -rf /var/lib/apt/lists/*
# Binary packages omit build-only dependencies (e.g. angles); use Debian source metadata.
RUN apt-get update && apt-get build-dep -y \
      ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs ros-jazzy-libmavconn && \
    rm -rf /var/lib/apt/lists/*
ARG MAVROS_COMMIT=22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e
RUN git clone --branch 2.15.1 --depth 1 https://github.com/mavlink/mavros.git /opt/mavros_ws/src/mavros && \
    test "$(git -C /opt/mavros_ws/src/mavros rev-parse HEAD)" = "$MAVROS_COMMIT" && \
    source /opt/ros/jazzy/setup.bash && cd /opt/mavros_ws && \
    CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
ENV HOME=/tmp/solo-home RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONUNBUFFERED=1
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/solo-entrypoint"]

# Keep application changes after expensive dependency/compiler layers.
FROM common AS application
COPY ros_ws/src /opt/solo_ws/src
RUN source /opt/ros/jazzy/setup.bash && source /opt/mavros_ws/install/setup.bash && cd /opt/solo_ws && \
    colcon build --symlink-install --event-handlers console_direct+
COPY config /workspace/config
COPY ros_ws/src /workspace/ros_ws/src
COPY locks/versions.json /opt/solo/versions.json
COPY docker/entrypoint.sh /usr/local/bin/solo-entrypoint
RUN chmod +x /usr/local/bin/solo-entrypoint

FROM application AS companion
RUN mkdir -p /opt/solo && \
    dpkg-query -W -f='${binary:Package}=${Version}\n' | sort > /opt/solo/apt-packages.lock
CMD ["ros2", "launch", "/workspace/ros_ws/src/solo_bringup/launch/hardware.launch.py"]

FROM common AS simulation_dependencies
ARG ARDUPILOT_COMMIT=ee14ff3822161368ec403700bf96d02b84def06c
ARG GAZEBO_PLUGIN_COMMIT=082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
ARG BUILD_JOBS=4
COPY --from=uv /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/python UV_CACHE_DIR=/tmp/uv-cache
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic libgz-sim8-dev rapidjson-dev libopencv-dev \
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
      libegl1-mesa-dev libgl1-mesa-dri mesa-utils gawk ccache \
      libtool libtool-bin libxml2-dev libxslt1-dev && \
    rm -rf /var/lib/apt/lists/*
COPY locks/ardupilot-py311.lock /opt/solo/ardupilot-py311.lock
COPY locks/build-backend.lock /opt/solo/build-backend.lock
RUN uv python install 3.11.16 && \
    uv venv --python 3.11.16 /opt/venvs/ardupilot && \
    uv pip install --python /opt/venvs/ardupilot/bin/python --require-hashes \
      -r /opt/solo/build-backend.lock && \
    uv pip install --python /opt/venvs/ardupilot/bin/python --require-hashes \
      --no-build-isolation -r /opt/solo/ardupilot-py311.lock && rm -rf /tmp/uv-cache
RUN git clone --branch Copter-4.5.3 --depth 1 https://github.com/ArduPilot/ardupilot.git /opt/ardupilot && \
    test "$(git -C /opt/ardupilot rev-parse HEAD)" = "$ARDUPILOT_COMMIT" && \
    git -C /opt/ardupilot submodule update --init --recursive --depth 1 && \
    cd /opt/ardupilot && /opt/venvs/ardupilot/bin/python ./waf configure --board sitl && \
    /opt/venvs/ardupilot/bin/python ./waf copter -j"$BUILD_JOBS"
# gz-transport's exported target needs zmq.hpp; it is only an APT recommendation.
RUN apt-get -o Acquire::Retries=3 update --error-on=any && \
    apt-get -o Acquire::Retries=3 install -y --no-install-recommends cppzmq-dev && \
    rm -rf /var/lib/apt/lists/* && \
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git /opt/ardupilot_gazebo && \
    git -C /opt/ardupilot_gazebo checkout "$GAZEBO_PLUGIN_COMMIT" && \
    source /opt/ros/jazzy/setup.bash && export GZ_VERSION=harmonic && \
    cmake -S /opt/ardupilot_gazebo -B /opt/ardupilot_gazebo/build \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3 && \
    cmake --build /opt/ardupilot_gazebo/build -j"$BUILD_JOBS"
# Gazebo's Qt GUI uses SVG icons; --no-install-recommends does not pull this in.
RUN apt-get -o Acquire::Retries=3 update --error-on=any && \
    apt-get -o Acquire::Retries=3 install -y --no-install-recommends libqt5svg5 && \
    rm -rf /var/lib/apt/lists/*
RUN dpkg-query -W -f='${binary:Package}=${Version}\n' | sort > /opt/solo/apt-packages.lock && \
    git -C /opt/ardupilot submodule status --recursive > /opt/solo/submodules.lock && \
    sha256sum /usr/share/GeographicLib/geoids/egm96-5.pgm > /opt/solo/geoid.sha256 && \
    test -x /opt/ardupilot/build/sitl/bin/arducopter && \
    test -f /opt/ardupilot_gazebo/build/libArduPilotPlugin.so
FROM simulation_dependencies AS simulation
COPY --from=application /opt/solo_ws /opt/solo_ws
COPY --from=application /workspace /workspace
COPY --from=application /opt/solo/versions.json /opt/solo/versions.json
COPY --from=application /usr/local/bin/solo-entrypoint /usr/local/bin/solo-entrypoint
CMD ["ros2", "launch", "/workspace/ros_ws/src/solo_bringup/launch/simulation.launch.py"]

# Keep numerical / bridge dependencies independent of live application sources.
FROM simulation_dependencies AS simulation_mpc_dependencies
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz-bridge python3-opencv && rm -rf /var/lib/apt/lists/*
COPY locks/mpc-requirements.txt /opt/solo/mpc-requirements.txt
COPY locks/mpc-py312.lock /opt/solo/mpc-py312.lock
RUN python3 -m venv --system-site-packages /opt/venvs/mpc && \
    uv pip install --python /opt/venvs/mpc/bin/python --require-hashes -r /opt/solo/mpc-py312.lock
ENV PYTHONPATH=/opt/venvs/mpc/lib/python3.12/site-packages \
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 NUMBA_NUM_THREADS=1
FROM simulation_mpc_dependencies AS simulation_mpc
COPY --from=application /opt/solo_ws /opt/solo_ws
COPY --from=application /workspace /workspace
COPY --from=application /opt/solo/versions.json /opt/solo/versions.json
COPY --from=application /usr/local/bin/solo-entrypoint /usr/local/bin/solo-entrypoint
RUN source /opt/ros/jazzy/setup.bash && source /opt/mavros_ws/install/setup.bash && \
    cd /opt/solo_ws && colcon build --symlink-install --event-handlers console_direct+ && \
    dpkg-query -W -f='${binary:Package}=${Version}\n' | sort > /opt/solo/apt-packages.lock
CMD ["ros2", "launch", "/workspace/ros_ws/src/solo_bringup/launch/simulation.launch.py"]
