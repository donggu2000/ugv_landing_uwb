#!/usr/bin/env bash
set -eo pipefail

cd /home/hyeon/ugv_landing_sim
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=10.99.40.20:11811

ros2 daemon stop >/dev/null 2>&1 || true

exec ros2 launch ugv_landing_sim bunker_uwb_viz.launch.py use_tag:=false
