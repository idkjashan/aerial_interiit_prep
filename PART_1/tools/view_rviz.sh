#!/usr/bin/env bash
# Launch RViz2 with pre-configured UAV display
source /opt/ros/humble/setup.bash
source "$HOME/px4_ros_ws/install/setup.bash"
export ROS_DOMAIN_ID=77
RVIZ_CFG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/src/uav_vision/config/uav_vision.rviz"
rviz2 -d "$RVIZ_CFG"
