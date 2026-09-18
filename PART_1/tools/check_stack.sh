#!/usr/bin/env bash
# Quick triage while the sim is running: is every link in the chain alive?
# Uses the same ROS_DOMAIN_ID as run_sim.sh.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_WS="${PX4_WS:-$HOME/px4_ros_ws}"
source /opt/ros/humble/setup.bash
[ -f "$PX4_WS/install/setup.bash" ] && source "$PX4_WS/install/setup.bash"
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"

LOC_TOPIC="/fmu/out/vehicle_local_position"
ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_local_position_v1" && LOC_TOPIC="/fmu/out/vehicle_local_position_v1"

echo "=== topic rates (5 s each; 0.0 means BROKEN) ==="
for t in /uav/rgb /uav/depth /uav/camera_info /fmu/out/vehicle_attitude \
         /fmu/out/sensor_combined "$LOC_TOPIC" \
         /uav_visual_odometry/odom /fmu/in/vehicle_visual_odometry; do
  printf '%-42s ' "$t"
  timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' || echo 'NO DATA'
done
echo
echo "=== QoS of a PX4 output topic (durability must match qos.py) ==="
ros2 topic info /fmu/out/vehicle_attitude --verbose 2>/dev/null | grep -A3 'Publishers\|Durability\|Reliability' | head -20
echo
echo "=== is EKF2 actually fusing vision? ==="
timeout 4 ros2 topic echo /fmu/out/estimator_status_flags --once 2>/dev/null \
  | grep -E 'cs_ev_pos|cs_ev_vel|cs_ev_hgt|cs_gps_hgt|cs_inertial_dead_reckoning'
echo
echo "=== EKF2 position validity and accuracy ==="
timeout 4 ros2 topic echo "$LOC_TOPIC" --once 2>/dev/null \
  | grep -E '^(xy_valid|z_valid|v_xy_valid|eph|evh|dead_reckoning|x|y|z):'
echo
echo "=== EKF2 innovation test ratios (< 0.50 is passing) ==="
timeout 4 ros2 topic echo /fmu/out/estimator_status --once 2>/dev/null \
  | grep -E '(pos_test_ratio|vel_test_ratio|hgt_test_ratio|pos_horiz_accuracy|pos_vert_accuracy):'
echo
echo "=== vision health ==="
timeout 4 ros2 topic echo /uav_vision_health/status --once 2>/dev/null | grep -E 'key|value' | paste - - | head -25
