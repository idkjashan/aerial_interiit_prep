#!/usr/bin/env bash
# Start the Part 1 simulation in manual mode: MicroXRCEAgent (port 8890, ROS_DOMAIN_ID 77),
# PX4 SITL with the x500_depth_down in the vo_ground world, the clock and camera bridges,
# and the VO stack without the mission node. The vehicle waits on the ground for
# manual_control.py. Logs go to PART_1/logs/.
#   ./launch_sim.sh              with the Gazebo window
#   ./launch_sim.sh --headless
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_WS="${PX4_WS:-$HOME/px4_ros_ws}"
WORLD="${WORLD:-vo_ground}"
MODEL="${MODEL:-x500_depth_down}"
LOGS="$WS_DIR/logs"; mkdir -p "$LOGS"

HEADLESS=0

while [[ $# -gt 0 ]]; do
  case $1 in
    --headless)
      HEADLESS=1
      shift
      ;;
    --world|-w)
      WORLD="$2"
      shift 2
      ;;
    *)
      shift
      ;;
  esac
done

export ROS_DOMAIN_ID=77
export DISPLAY="${DISPLAY:-:0}"
unset LIBGL_ALWAYS_SOFTWARE
unset MESA_GL_VERSION_OVERRIDE
if command -v nvidia-smi >/dev/null 2>&1; then
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __VK_LAYER_NV_optimus=NVIDIA_only
fi

# Source ROS 2 environments
source /opt/ros/humble/setup.bash
source "$PX4_WS/install/setup.bash"
[[ -f "$WS_DIR/install/setup.bash" ]] && source "$WS_DIR/install/setup.bash"
ros2 daemon stop >/dev/null 2>&1 || true

# Gazebo resource paths
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"

if [[ ! -d "$PX4_DIR" ]]; then
  echo "================================================================"
  echo " [ERROR] PX4 directory not found at '$PX4_DIR'."
  echo " Please set the PX4_DIR environment variable, e.g.:"
  echo "   export PX4_DIR=/path/to/PX4-Autopilot"
  echo "================================================================"
  exit 1
fi

if [[ ! -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
  echo "================================================================"
  echo " [ERROR] PX4 SITL binary not found at '$PX4_DIR/build/px4_sitl_default/bin/px4'."
  echo " Please build PX4 SITL first by running:"
  echo "   cd $PX4_DIR && make px4_sitl"
  echo "================================================================"
  exit 1
fi

# Ensure airframe 4022 and simulation models are synced into PX4_DIR
if [[ ! -f "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_depth_down" ]]; then
  echo "[!] Copying Part 1 airframe 4022 into $PX4_DIR..."
  cp -r "$WS_DIR/px4/ROMFS"/* "$PX4_DIR/ROMFS/"
fi

if [[ ! -f "$PX4_DIR/Tools/simulation/gz/models/textured_ground/materials/textures/ground_albedo.png" ]]; then
  echo "[!] Copying Part 1 models and textures into $PX4_DIR..."
  mkdir -p "$PX4_DIR/Tools/simulation/gz/models/textured_ground/materials/textures"
  cp -r "$WS_DIR/px4/Tools"/* "$PX4_DIR/Tools/"
fi

# Clean stale parameters and leftover bridges
rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/parameters"*.bson
pkill -9 -f "parameter_bridge" 2>/dev/null || true

PX4_INSTANCE=1
PX4_PORT=8890
MODEL_INSTANCE="${MODEL}_${PX4_INSTANCE}"
UAV_POSE="0,0,0.25,0,0,0"

export PX4_UXRCE_DDS_PORT="$PX4_PORT"
export PX4_UXRCE_DDS_NS=""

pids=()
cleanup() {
  echo
  echo "=================================================="
  echo " Shutting down simulation and nodes cleanly...    "
  echo "=================================================="
  for pid in "${pids[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${pids[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
    pkill -P "$pid" 2>/dev/null || true
  done
  pkill -x px4 2>/dev/null || true
  pkill -9 -f "gz sim" 2>/dev/null || true
  pkill -x gz-sim-server 2>/dev/null || true
  pkill -x gz-sim-gui 2>/dev/null || true
  pkill -x MicroXRCEAgent 2>/dev/null || true
  pkill -f "parameter_bridge" 2>/dev/null || true
  pkill -f "uav_vision" 2>/dev/null || true
  sleep 1
  for pid in "${pids[@]}"; do
    kill -KILL "$pid" 2>/dev/null || true
  done
  echo "Shutdown complete."
}
trap cleanup EXIT INT TERM

echo "================================================================"
echo "  Starting GPS-Denied UAV Simulation (Manual Flight Mode)       "
echo "================================================================"
echo "  World:       $WORLD"
echo "  Model:       $MODEL_INSTANCE"
echo "  GUI:         $([ "$HEADLESS" -eq 1 ] && echo 'Disabled (Headless)' || echo 'Enabled (Gazebo Window)')"
echo "  DDS Port:    $PX4_PORT (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "  MAVLink:     UDP 14550 / 18571 (QGroundControl Auto-Connect)"
echo "================================================================"

echo "[1/4] Starting MicroXRCEAgent..."
MicroXRCEAgent udp4 -p "$PX4_PORT" >"$LOGS/agent.log" 2>&1 & pids+=($!)
sleep 2

echo "[2/4] Starting PX4 SITL + Gazebo (Spawning $MODEL_INSTANCE)..."
( cd "$PX4_DIR" && \
    unset PX4_GZ_STANDALONE && \
    { [[ "$HEADLESS" -eq 1 ]] && export HEADLESS=1 || unset HEADLESS; } && \
    PX4_SYS_AUTOSTART=4022 \
    PX4_GZ_MODEL_POSE="$UAV_POSE" \
    PX4_SIM_MODEL="gz_$MODEL" PX4_GZ_WORLD="$WORLD" \
    ./build/px4_sitl_default/bin/px4 -d -i "$PX4_INSTANCE" ) >"$LOGS/px4.log" 2>&1 & pids+=($!)

echo "      Waiting for PX4 attitude telemetry..."
for i in $(seq 1 60); do
  ros2 topic list 2>/dev/null | grep -q '/fmu/out/vehicle_attitude' && break; sleep 1
done

echo "[3/4] Starting ROS-Gazebo Bridges (Clock & Camera)..."
# Clock bridge
ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  --ros-args -r "/world/$WORLD/clock:=/clock" \
  >"$LOGS/clock_bridge.log" 2>&1 & pids+=($!)

# Camera bridge (isolated so pausing camera does not pause clock)
ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/model/${MODEL_INSTANCE}/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/world/$WORLD/model/${MODEL_INSTANCE}/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  --ros-args \
  -r "/world/$WORLD/model/${MODEL_INSTANCE}/link/camera_link/sensor/IMX214/image:=/uav/rgb" \
  -r "/world/$WORLD/model/${MODEL_INSTANCE}/link/camera_link/sensor/IMX214/camera_info:=/uav/camera_info" \
  -r "/depth_camera:=/uav/depth" \
  -r "/depth_camera/points:=/uav/points" \
  >"$LOGS/bridge.log" 2>&1 & pids+=($!)

echo "      Waiting for camera images on /uav/rgb..."
for i in $(seq 1 30); do
  timeout 3 ros2 topic echo --once /uav/rgb >/dev/null 2>&1 && break || true
  sleep 1
done

echo "[4/4] Starting Visual Odometry Pipeline (Perception Only)..."
ros2 launch uav_vision gps_denied_vo.launch.py run_mission:=false \
  >"$LOGS/vision.log" 2>&1 & pids+=($!)
sleep 2

echo
echo "================================================================"
echo "  SIMULATION READY (MANUAL FLIGHT MODE)                         "
echo "================================================================"
echo "  The drone is spawned and sitting on the ground.               "
echo "  Visual Odometry and EKF2 fusion are active.                   "
echo
echo "  Next steps in separate terminals:                             "
echo "  1. View HUD Monitor:    python3 tools/monitor.py              "
echo "  2. Flight Control:      python3 tools/manual_control.py       "
echo "     (or CLI):            python3 tools/manual_control.py takeoff 10"
echo "  3. Teleoperation:       ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "  4. QGroundControl:      Connects automatically on UDP 14550   "
echo "  5. Test Vision Cut:     ./tools/vision_cut.sh test 5          "
echo "================================================================"
echo "Press Ctrl-C to stop simulation."
wait
