FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    curl \
    git \
    gnupg \
    iputils-ping \
    less \
    mesa-utils \
    nano \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-rviz2 \
    ros-humble-tf-transformations \
    ros-humble-xacro \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'if [ -f /workspaces/ugv_landing_sim/install/setup.bash ]; then source /workspaces/ugv_landing_sim/install/setup.bash; fi' \
    'export GZ_SIM_RESOURCE_PATH=/workspaces/ugv_landing_sim/src/ugv_landing_sim/worlds:/workspaces/ugv_landing_sim/src/ugv_landing_sim/models:${GZ_SIM_RESOURCE_PATH}' \
    'export IGN_GAZEBO_RESOURCE_PATH=/workspaces/ugv_landing_sim/src/ugv_landing_sim/worlds:/workspaces/ugv_landing_sim/src/ugv_landing_sim/models:${IGN_GAZEBO_RESOURCE_PATH}' \
    >> /root/.bashrc

WORKDIR /workspaces/ugv_landing_sim

CMD ["/bin/bash"]
