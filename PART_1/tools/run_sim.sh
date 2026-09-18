#!/usr/bin/env bash
# Bring up the GPS-denied VO stack in the right order.
#   ./run_sim.sh                  start everything and fly the 90 s hover
#   ./run_sim.sh --no-mission     perception only (no arming, no flight)
#   ./run_sim.sh --gui --world baylands
# Each component logs to its own file under PART_1/logs/.
set -eo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WS_DIR="${WS_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"   # PART_1/
PX4_WS="${PX4_WS:-$HOME/px4_ros_ws}"
WORLD="${WORLD:-vo_ground}"
MODEL="${MODEL:-x500_depth_down}"
LOGS="$WS_DIR/logs"; mkdir -p "$LOGS"
RUN_MISSION=true
HEADLESS=1

# Parse command line options
while [[ $# -gt 0 ]]; do
  case $1 in
    --no-mission)
      RUN_MISSION=false
      shift
      ;;
    --gui)
      HEADLESS=0
      shift
      ;;
    --world|-w)
      WORLD="$2"
      shift 2
      ;;
    --pose|-p)
      CUSTOM_POSE="$2"
      shift 2
      ;;
    *)
      shift
      ;;
  esac
done

# World-specific terrain spawn offsets
case ${WORLD%_overlay} in
  drdo_world1)
    DEFAULT_POSE="-10.226,311.831,22.863,0.011338,0.135709,-2.161422"
    ;;
  drdo_world2)
    DEFAULT_POSE="103.776917,-101.472992,17.318562,-0.054656,0.032451,2.460081"
    ;;
  drdo_world3)
    DEFAULT_POSE="108.849,-265.663,49.4752,0.045161,0.003268,1.588"
    ;;
  vo_ground|*)
    DEFAULT_POSE="0,0,0.25,0,0,0"
    ;;
esac

UAV_POSE="${CUSTOM_POSE:-$DEFAULT_POSE}"

# Isolate our simulation session from other concurrent sessions
export ROS_DOMAIN_ID=77
export GZ_PARTITION="uav_sim_$$"
export DISPLAY="${DISPLAY:-:0}"

source /opt/ros/humble/setup.bash
source "$PX4_WS/install/setup.bash"
[[ -f "$WS_DIR/install/setup.bash" ]] && source "$WS_DIR/install/setup.bash"
ros2 daemon stop >/dev/null 2>&1 || true

export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only
# extra world/model packages: add them to GZ_SIM_RESOURCE_PATH before running
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# Clean stale PX4 parameters so airframe 4022 defaults always apply
rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/parameters"*.bson

# Instance and port isolation
PX4_INSTANCE="${PX4_INSTANCE:-1}"
PX4_PORT="${PX4_PORT:-8890}"
MODEL_INSTANCE="${MODEL}_${PX4_INSTANCE}"
export PX4_UXRCE_DDS_PORT="$PX4_PORT"
export PX4_UXRCE_DDS_NS=""

pids=()
cleanup() {
  echo; echo "shutting down our session..."
  for pid in "${pids[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${pids[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
    pkill -P "$pid" 2>/dev/null || true
  done
  pkill -f "$PX4_DIR/build/px4_sitl_default/bin/px4.*-i $PX4_INSTANCE" 2>/dev/null || true
  pkill -f "MicroXRCEAgent udp4 -p $PX4_PORT" 2>/dev/null || true
  pkill -f "parameter_bridge.*$WORLD" 2>/dev/null || true
  sleep 1
  for pid in "${pids[@]}"; do
    kill -KILL "$pid" 2>/dev/null || true
    pkill -9 -P "$pid" 2>/dev/null || true
  done
  pkill -9 -f "$PX4_DIR/build/px4_sitl_default/bin/px4.*-i $PX4_INSTANCE" 2>/dev/null || true
  pkill -9 -f "MicroXRCEAgent udp4 -p $PX4_PORT" 2>/dev/null || true
  pkill -9 -f "parameter_bridge.*$WORLD" 2>/dev/null || true
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "[1/5] MicroXRCEAgent (port=$PX4_PORT, ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
MicroXRCEAgent udp4 -p "$PX4_PORT" >"$LOGS/agent.log" 2>&1 & pids+=($!)
sleep 2

echo "[2/5] PX4 SITL + Gazebo ($MODEL_INSTANCE in $WORLD at $UAV_POSE, partition $GZ_PARTITION)"
( cd "$PX4_DIR" && \
    unset PX4_GZ_STANDALONE && \
    unset HEADLESS && \
    PX4_SYS_AUTOSTART=4022 \
    PX4_GZ_MODEL_POSE="$UAV_POSE" \
    PX4_SIM_MODEL="gz_$MODEL" PX4_GZ_WORLD="$WORLD" \
    ./build/px4_sitl_default/bin/px4 -d -i "$PX4_INSTANCE" ) >"$LOGS/px4.log" 2>&1 & pids+=($!)
echo "      waiting for PX4 to publish /fmu/out/vehicle_attitude ..."
for i in $(seq 1 60); do
  ros2 topic list 2>/dev/null | grep -q '/fmu/out/vehicle_attitude' && break; sleep 1
done

echo "[3/5] ros_gz_bridge (directional GZ->ROS for $MODEL_INSTANCE)"
ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/model/${MODEL_INSTANCE}/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/world/$WORLD/model/${MODEL_INSTANCE}/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  "/world/$WORLD/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  --ros-args \
  -r "/world/$WORLD/clock:=/clock" \
  -r "/world/$WORLD/model/${MODEL_INSTANCE}/link/camera_link/sensor/IMX214/image:=/uav/rgb" \
  -r "/world/$WORLD/model/${MODEL_INSTANCE}/link/camera_link/sensor/IMX214/camera_info:=/uav/camera_info" \
  -r "/depth_camera:=/uav/depth" \
  -r "/depth_camera/points:=/uav/points" \
  >"$LOGS/bridge.log" 2>&1 & pids+=($!)

echo "      waiting for camera images on /uav/rgb ..."
for i in $(seq 1 30); do
  timeout 3 ros2 topic echo --once /uav/rgb >/dev/null 2>&1 && break || true
  sleep 1
done

echo "[4/5] vision stack (run_mission=$RUN_MISSION)"
ros2 launch uav_vision gps_denied_vo.launch.py run_mission:="$RUN_MISSION" \
  >"$LOGS/vision.log" 2>&1 & pids+=($!)
sleep 2

echo "[5/5] recording rosbag"
ros2 bag record -o "$LOGS/bag_$(date +%Y%m%d_%H%M%S)" \
  /uav_visual_odometry/odom /uav_visual_odometry/quality /uav_visual_odometry/health \
  /uav_visual_odometry/agl /diagnostics /uav_vision_health/status \
  /fmu/out/vehicle_local_position /fmu/out/vehicle_local_position_v1 \
  /fmu/out/vehicle_status /fmu/out/vehicle_status_v1 \
  /fmu/out/estimator_status_flags /fmu/out/estimator_status \
  /fmu/in/vehicle_visual_odometry \
  >"$LOGS/bag.log" 2>&1 & pids+=($!)

echo
echo "up. logs in $LOGS ; follow with:  tail -f $LOGS/vision.log"
echo "Ctrl-C to stop everything."
wait

