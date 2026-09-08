#!/usr/bin/env bash
# Bring up the GPS-denied VO stack in the correct order.
#   ./run_sim.sh            start everything
#   ./run_sim.sh --no-mission   bring up perception only (no arming / no flight)
# Each component runs in its own log file under /tmp/uav_vision_logs.
set -uo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WS_DIR="${WS_DIR:-$HOME/aerial_interiit_prep/PART_1}"
PX4_WS="${PX4_WS:-$HOME/px4_ros_ws}"
WORLD="${WORLD:-default}"
MODEL="${MODEL:-x500_depth_down}"
LOGS=/tmp/uav_vision_logs; mkdir -p "$LOGS"
RUN_MISSION=true; [[ "${1:-}" == "--no-mission" ]] && RUN_MISSION=false

source /opt/ros/humble/setup.bash
source "$PX4_WS/install/setup.bash"
[[ -f "$WS_DIR/install/setup.bash" ]] && source "$WS_DIR/install/setup.bash"

export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only

pids=()
cleanup() { echo; echo "shutting down..."; kill "${pids[@]}" 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT INT TERM

echo "[1/5] MicroXRCEAgent"
MicroXRCEAgent udp4 -p 8888 >"$LOGS/agent.log" 2>&1 & pids+=($!)
sleep 2

echo "[2/5] PX4 SITL + Gazebo ($MODEL in $WORLD)"
( cd "$PX4_DIR" && PX4_GZ_STANDALONE=0 PX4_SIM_MODEL="gz_$MODEL" PX4_GZ_WORLD="$WORLD" \
    ./build/px4_sitl_default/bin/px4 ) >"$LOGS/px4.log" 2>&1 & pids+=($!)
echo "      waiting for PX4 to publish /fmu/out/vehicle_attitude ..."
for i in $(seq 1 60); do
  ros2 topic list 2>/dev/null | grep -q '/fmu/out/vehicle_attitude' && break; sleep 1
done

echo "[3/5] ros_gz_bridge"
ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/model/${MODEL}_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image" \
  "/world/$WORLD/model/${MODEL}_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
  "/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image" \
  "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
  "/world/$WORLD/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  --ros-args \
  -r "/world/$WORLD/clock:=/clock" \
  -r "/world/$WORLD/model/${MODEL}_0/link/camera_link/sensor/IMX214/image:=/uav/rgb" \
  -r "/world/$WORLD/model/${MODEL}_0/link/camera_link/sensor/IMX214/camera_info:=/uav/camera_info" \
  -r "/depth_camera:=/uav/depth" \
  -r "/depth_camera/camera_info:=/uav/depth/camera_info" \
  >"$LOGS/bridge.log" 2>&1 & pids+=($!)
sleep 3

echo "[4/5] vision stack (run_mission=$RUN_MISSION)"
ros2 launch uav_vision gps_denied_vo.launch.py run_mission:="$RUN_MISSION" \
  >"$LOGS/vision.log" 2>&1 & pids+=($!)
sleep 2

echo "[5/5] recording rosbag"
ros2 bag record -o "$LOGS/bag_$(date +%Y%m%d_%H%M%S)" \
  /uav_visual_odometry/odom /uav_visual_odometry/quality /uav_visual_odometry/health \
  /uav_visual_odometry/agl /diagnostics /uav_vision_health/status \
  /fmu/out/vehicle_local_position /fmu/out/vehicle_status \
  /fmu/out/estimator_status_flags /fmu/in/vehicle_visual_odometry \
  >"$LOGS/bag.log" 2>&1 & pids+=($!)

echo
echo "up. logs in $LOGS ; follow with:  tail -f $LOGS/vision.log"
echo "Ctrl-C to stop everything."
wait
