# ROS2 Humble image for the lmpc_ros2 stack: builds this repo's lmpc_ros2
# (C++ rclcpp node) plus f1tenth_gym_ros from source against Humble --
# upstream f1tenth_gym_ros pins ROS2 Foxy, but lmpc_ros2's rclcpp usage
# assumes Humble/Jazzy-era APIs, so f1tenth_gym_ros (pure rclpy, no C++) is
# the side rebuilt here rather than the other way around.
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      gdb \
      git \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

WORKDIR /ws/src

# This repo (contains lmpc_ros2, the vendored f1tenth_gym, src_gym, etc.)
COPY . /ws/src/LearningMPC

# The repo root is itself a legacy ROS1/catkin package (package.xml declares
# <buildtool_depend>catkin</buildtool_depend>, which has no ROS2 rosdep
# mapping and breaks rosdep/colcon's recursive package scan). Out of scope
# for this image -- drop the manifest/build files so only the nested
# lmpc_ros2 package.xml is discovered.
RUN rm -f /ws/src/LearningMPC/package.xml /ws/src/LearningMPC/CMakeLists.txt

# f1tenth_gym_ros itself -- not vendored in this repo, built from source
# against Humble instead of its official ros:foxy pin.
RUN git clone --depth 1 https://github.com/f1tenth/f1tenth_gym_ros.git \
      /ws/src/f1tenth_gym_ros

# sim.yaml hardcodes /sim_ws/src/f1tenth_gym_ros/maps/... (upstream's own
# Docker convention) as an absolute map_path -- symlink it into place
# rather than patching their config.
RUN mkdir -p /sim_ws/src && ln -s /ws/src/f1tenth_gym_ros /sim_ws/src/f1tenth_gym_ros

# Default the sim to lmpc_ros2's own gold_conference_room track instead of
# upstream's bundled Levine map -- Levine has no relation to the waypoints/
# safe-set lmpc_ros2 is actually driving, which makes every default run
# QP-infeasible. Unlike barc_oval (start pose ~(0,0,0), matching sim.yaml's
# own sx/sy/stheta defaults), gold_conference_room's centerline starts at
# roughly (-3.396, -11.568, 1.774 rad) -- sx/sy/stheta must be patched too,
# not just map_path.
RUN cp /ws/src/LearningMPC/lmpc_ros2/data/gold_conference_room/gold_conference_room_map.png \
       /ws/src/LearningMPC/lmpc_ros2/data/gold_conference_room/gold_conference_room_map.yaml \
       /ws/src/f1tenth_gym_ros/maps/ && \
    sed -i \
      -e "s#maps/levine#maps/gold_conference_room_map#" \
      -e "s/^\(\s*sx:\s*\).*/\1-3.3957536/" \
      -e "s/^\(\s*sy:\s*\).*/\1-11.567745/" \
      -e "s/^\(\s*stheta:\s*\).*/\11.773677/" \
      /ws/src/f1tenth_gym_ros/config/sim.yaml

# Eigen 3.4.0 / OSQP v0.6.3 / osqp-eigen v0.8.1 into src_gym/deps, same
# versions and steps as src_gym/build.sh's dependency stage -- lmpc_ros2's
# CMakeLists.txt looks for them there via LMPC_DEPS_DIR. Skips build.sh's
# own pybind11 module stage, which lmpc_ros2 (a plain rclcpp executable,
# no Python in the control loop) doesn't need.
RUN set -eux; \
    DEPS=/ws/src/LearningMPC/src_gym/deps; SRC="$DEPS/src"; mkdir -p "$SRC"; \
    git clone --depth 1 --branch 3.4.0 https://gitlab.com/libeigen/eigen.git "$SRC/eigen"; \
    cmake -S "$SRC/eigen" -B "$SRC/eigen/build" -DCMAKE_INSTALL_PREFIX="$DEPS" \
          -DEIGEN_BUILD_DOC=OFF -DBUILD_TESTING=OFF -DEIGEN_BUILD_PKGCONFIG=OFF; \
    cmake --install "$SRC/eigen/build"; \
    git clone --depth 1 --branch v0.6.3 --recursive https://github.com/osqp/osqp.git "$SRC/osqp"; \
    cmake -S "$SRC/osqp" -B "$SRC/osqp/build" -DCMAKE_INSTALL_PREFIX="$DEPS" \
          -DCMAKE_BUILD_TYPE=Release -DUNITTESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON; \
    cmake --build "$SRC/osqp/build" -j"$(nproc)"; \
    cmake --install "$SRC/osqp/build"; \
    git clone --depth 1 --branch v0.8.1 https://github.com/robotology/osqp-eigen.git "$SRC/osqp-eigen"; \
    cmake -S "$SRC/osqp-eigen" -B "$SRC/osqp-eigen/build" -DCMAKE_INSTALL_PREFIX="$DEPS" \
          -DCMAKE_PREFIX_PATH="$DEPS" -DCMAKE_BUILD_TYPE=Release \
          -DOSQP_EIGEN_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON; \
    cmake --build "$SRC/osqp-eigen/build" -j"$(nproc)"; \
    cmake --install "$SRC/osqp-eigen/build"; \
    rm -rf "$SRC"

# lmpc_node links osqp/osqp-eigen from src_gym/deps/lib, which isn't on the
# default runtime linker search path -- register it so the built
# executable can actually load at runtime.
RUN echo "/ws/src/LearningMPC/src_gym/deps/lib" > /etc/ld.so.conf.d/lmpc-deps.conf && ldconfig

# f1tenth_gym (vendored at LearningMPC/f1tenth_gym) needs the legacy-pip
# dance documented in requirements.txt's comments -- gym==0.19.0 has
# metadata pip>=24.1 rejects. transforms3d is f1tenth_gym_ros's own
# rosdep-unresolvable pip build dependency.
RUN python3 -m pip install --no-cache-dir --upgrade "pip==24.0" "setuptools==65.5.0" "wheel==0.38.4" && \
    python3 -m pip install --no-cache-dir "gym==0.19.0" --no-build-isolation && \
    python3 -m pip install --no-cache-dir -e /ws/src/LearningMPC/f1tenth_gym --no-deps && \
    python3 -m pip install --no-cache-dir -r /ws/src/LearningMPC/requirements.txt && \
    python3 -m pip install --no-cache-dir transforms3d

WORKDIR /ws

RUN apt-get update && \
    . /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys="gym f1tenth_gym" && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/humble/setup.bash && \
    colcon build --packages-select f1tenth_gym_ros lmpc_ros2 \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

COPY lmpc_ros2/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
