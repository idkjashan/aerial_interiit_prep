#!/usr/bin/env bash
# Start the Part 2 simulation in the background and wait on the ground:
# MicroXRCEAgent (port 8890, ROS_DOMAIN_ID 77), PX4 SITL with the Gazebo x500 (airframe
# 4001) and the Part 2 parameters, and a clock bridge. MAVLink for QGC on 14550 and for
# the scripts on 14540. Logs go to PART_2/logs/.
#   ./launch_sim.sh              with the Gazebo window
#   ./launch_sim.sh --headless
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_WS="${PX4_WS:-$HOME/px4_ros_ws}"
WORLD="${WORLD:-default}"
MODEL="${MODEL:-x500}"
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

# GPU acceleration on NVIDIA RTX 4060
unset LIBGL_ALWAYS_SOFTWARE
unset MESA_GL_VERSION_OVERRIDE
if command -v nvidia-smi >/dev/null 2>&1; then
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __VK_LAYER_NV_optimus=NVIDIA_only
fi

# Source ROS 2 environments
source /opt/ros/humble/setup.bash
[[ -f "$PX4_WS/install/setup.bash" ]] && source "$PX4_WS/install/setup.bash"
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

# Clean stale parameters
rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/parameters"*.bson

PX4_INSTANCE="${PX4_INSTANCE:-0}"
PX4_PORT=8890
MODEL_INSTANCE="gz_${MODEL}"
[[ "$PX4_INSTANCE" -gt 0 ]] && MODEL_INSTANCE="${MODEL_INSTANCE}_${PX4_INSTANCE}"
UAV_POSE="0,0,0.25,0,0,0"

export PX4_UXRCE_DDS_PORT="$PX4_PORT"
export PX4_UXRCE_DDS_NS=""

# Part 2 parameters applied at startup
export PX4_PARAM_MIS_TAKEOFF_ALT=20
export PX4_PARAM_CA_FAILURE_MODE=1      # Built-in motor failure handling
export PX4_PARAM_COM_ACT_FAIL_ACT=0     # Warning only on motor failure
export PX4_PARAM_NAV_DLL_ACT=0          # No RTL on GCS disconnect
export PX4_PARAM_SDLOG_PROFILE=147      # High-rate logging for actuator and attitude

pids=()
cleanup() {
  echo
  echo "=================================================="
  echo " Shutting down Part 2 simulation cleanly...       "
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
  pkill -f "parameter_bridge.*$WORLD" 2>/dev/null || true
  pkill -f "PART_2/tools/monitor.py" 2>/dev/null || true
  sleep 1
  for pid in "${pids[@]}"; do
    kill -KILL "$pid" 2>/dev/null || true
  done
  echo "Shutdown complete."
}
trap cleanup EXIT INT TERM

echo "================================================================"
echo "  Starting Part 2 UAV Simulation (Rotor Effectiveness)          "
echo "================================================================"
echo "  World:       $WORLD"
echo "  Model:       gz_$MODEL (Airframe 4001)"
echo "  GUI:         $([ "$HEADLESS" -eq 1 ] && echo 'Disabled (Headless)' || echo 'Enabled (Gazebo Window)')"
echo "  DDS Port:    $PX4_PORT (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "  MAVLink:     UDP 14550 (QGC) & UDP 14540 (Scripts)"
echo "================================================================"

echo "[1/3] Starting MicroXRCEAgent..."
MicroXRCEAgent udp4 -p "$PX4_PORT" >"$LOGS/agent.log" 2>&1 & pids+=($!)
sleep 2

echo "[2/3] Starting PX4 SITL + Gazebo (Spawning gz_$MODEL)..."
( cd "$PX4_DIR" && \
    unset PX4_GZ_STANDALONE && \
    { [[ "$HEADLESS" -eq 1 ]] && export HEADLESS=1 || unset HEADLESS; } && \
    PX4_SYS_AUTOSTART=4001 \
    PX4_GZ_MODEL_POSE="$UAV_POSE" \
    PX4_SIM_MODEL="gz_$MODEL" PX4_GZ_WORLD="$WORLD" \
    ./build/px4_sitl_default/bin/px4 -d $([ "$PX4_INSTANCE" -gt 0 ] && echo "-i $PX4_INSTANCE") ) >"$LOGS/px4.log" 2>&1 & pids+=($!)

echo "      Waiting for PX4 attitude telemetry..."
for i in $(seq 1 60); do
  ros2 topic list 2>/dev/null | grep -q '/fmu/out/vehicle_attitude' && break; sleep 1
done

echo "[3/3] Starting Clock Bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  --ros-args -r "/world/$WORLD/clock:=/clock" \
  >"$LOGS/clock_bridge.log" 2>&1 & pids+=($!)
sleep 1

echo
echo "================================================================"
echo "  PART 2 SIMULATION READY (MANUAL FLIGHT MODE)                  "
echo "================================================================"
echo "  The drone is spawned and sitting on the ground.               "
echo "  Control Allocator runtime effectiveness scaling is active.    "
echo
echo "  Next steps in separate terminals:                             "
echo "  1. View Telemetry HUD:  python3 PART_2/tools/monitor.py       "
echo "  2. Flight Control:      python3 PART_2/tools/manual_control.py takeoff 20"
echo "     (or QGC):            Slide to Arm & Takeoff to 20m in QGC  "
echo "  3. Scale Effectiveness: python3 PART_2/tools/set_effectiveness.py 0.75"
echo "     (levels):            0.75, 0.50, 0.25, 0.00, or failure    "
echo "================================================================"
echo "Press Ctrl-C to stop simulation."
wait
