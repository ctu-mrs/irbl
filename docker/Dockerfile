# irbl + PX4-Autopilot SITL (Gazebo Harmonic) on ROS 2 Jazzy, no MRS system.
#
# Build (from the irbl repo root, so the build context is this repo):
#   docker build -f docker/Dockerfile -t irbl:jazzy .
# Run: see docker/run.sh
#
# Layout inside the image mirrors what tmux/px4_sim_*/session.yml expects:
#   /root/git/PX4-Autopilot          (session.yml's default PX4_DIR=$HOME/git/PX4-Autopilot)
#   /root/irbl_ws/src/{irbl,px4_msgs,px4_ros_com}
#   /root/irbl_ws/install            (session.yml sources ../../../../install/setup.zsh)

FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# PX4 fork + commit, and the PX4-gazebo-models fork/branch that has cylinder_forest + x500_lidar_3d.
ARG PX4_REPO=https://github.com/manuelboldrer/PX4-Autopilot.git
ARG PX4_REF=ca1d36476aee21cb835df5dfacc7eb46f1fdf047
ARG GZ_MODELS_REPO=https://github.com/manuelboldrer/PX4-gazebo-models.git
ARG GZ_MODELS_REF=ebdd7a851d457fa944ea9d1efd1ffc1021806d37
ARG XRCE_AGENT_REF=v3.0.2

RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo git wget curl lsb-release gnupg ca-certificates \
      build-essential cmake ninja-build \
      tmux tmuxinator zsh vim less \
      python3-yaml python3-pip \
      libgl1-mesa-dri mesa-utils \
      ros-jazzy-ros-gz ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
      ros-jazzy-octomap ros-jazzy-octomap-msgs ros-jazzy-tf2-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------- Micro XRCE-DDS Agent
RUN git clone --depth 1 -b ${XRCE_AGENT_REF} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce \
    && cmake -S /tmp/xrce -B /tmp/xrce/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/xrce/build -j"$(nproc)" \
    && cmake --install /tmp/xrce/build \
    && ldconfig \
    && rm -rf /tmp/xrce

# ---------------------------------------------------------------- PX4-Autopilot (shallow)
# Only the pinned commit is fetched (depth 1), submodules too -- no history.
WORKDIR /root/git/PX4-Autopilot
RUN git init -q . \
    && git remote add origin ${PX4_REPO} \
    && git fetch -q --depth 1 origin ${PX4_REF} \
    && git checkout -q FETCH_HEAD \
    # Tools/simulation/gz comes from the gazebo-models fork instead of the gitlink recorded upstream
    && git submodule init \
    && git config submodule.Tools/simulation/gz.update none \
    && git submodule update --recursive --depth 1 --jobs 8 \
    && rm -rf Tools/simulation/gz && mkdir -p Tools/simulation/gz \
    && git -C Tools/simulation/gz init -q \
    && git -C Tools/simulation/gz remote add origin ${GZ_MODELS_REPO} \
    && git -C Tools/simulation/gz fetch -q --depth 1 origin ${GZ_MODELS_REF} \
    && git -C Tools/simulation/gz checkout -q FETCH_HEAD

# PX4 deps (+ Gazebo Harmonic from packages.osrfoundation.org), no NuttX toolchain.
RUN bash ./Tools/setup/ubuntu.sh --no-nuttx \
    && rm -rf /var/lib/apt/lists/*

# GIT_SUBMODULES_ARE_EVIL: don't let the build "fix" the swapped gz models submodule.
RUN GIT_SUBMODULES_ARE_EVIL=1 make px4_sitl \
    && mkdir -p build/px4_sitl_default/rootfs

# ---------------------------------------------------------------- ROS 2 workspace
WORKDIR /root/irbl_ws/src
# px4_msgs must match the PX4 checkout's message definitions (versioned topics), so take them from it.
RUN git clone --depth 1 https://github.com/PX4/px4_msgs.git \
    && git clone --depth 1 https://github.com/PX4/px4_ros_com.git \
    && rm -f px4_msgs/msg/*.msg px4_msgs/srv/*.srv \
    && cp /root/git/PX4-Autopilot/msg/*.msg px4_msgs/msg/ \
    && { cp /root/git/PX4-Autopilot/msg/versioned/*.msg px4_msgs/msg/ 2>/dev/null || true; } \
    && cp /root/git/PX4-Autopilot/srv/*.srv px4_msgs/srv/

# Build px4_msgs/px4_ros_com in their own layer: they are slow and rarely change.
WORKDIR /root/irbl_ws
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select px4_msgs px4_ros_com

COPY . /root/irbl_ws/src/irbl

# filter_reflective_uavs (needs mrs_lib/ouster_ros) is optional for rbl_controller_node and not included.
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
         --skip-keys "filter_reflective_uavs mrs_lib mrs_msgs mrs_uav_testing" \
    && rm -rf /var/lib/apt/lists/* \
    && source /opt/ros/jazzy/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# session.yml sources setup.zsh, so tmux panes must run zsh.
ENV SHELL=/bin/zsh
RUN for rc in .zshrc .bashrc; do \
      ext=${rc#.}; ext=${ext%rc}; \
      printf 'source /opt/ros/jazzy/setup.%s\nsource /root/irbl_ws/install/setup.%s\nexport PX4_DIR=/root/git/PX4-Autopilot\n' "$ext" "$ext" >> /root/$rc; \
    done

WORKDIR /root/irbl_ws/src/irbl/tmux
CMD ["/bin/zsh"]
