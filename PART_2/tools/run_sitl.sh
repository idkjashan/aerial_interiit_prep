#!/usr/bin/env bash
# Start PX4 SITL with the Gazebo x500 for the Part 2 experiments.
#   ./run_sitl.sh                 headless, default world
#   ./run_sitl.sh --gui           with the Gazebo window (for the video)
#   WORLD=vo_ground ./run_sitl.sh
# PX4 has to be built with the patches in ../px4/patches first (see ../README.md).
set -eo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WORLD="${WORLD:-default}"
[[ "${1:-}" == "--gui" ]] || export HEADLESS=1

# PX4 applies PX4_PARAM_* at startup (ROMFS/px4fmu_common/init.d-posix/rcS)
export PX4_PARAM_MIS_TAKEOFF_ALT=20
export PX4_PARAM_CA_FAILURE_MODE=1      # built-in motor failure handling, for the Phase 1 reference
export PX4_PARAM_COM_ACT_FAIL_ACT=0     # ...without a mode switch when it triggers
export PX4_PARAM_NAV_DLL_ACT=0          # no RTL when fly.py disconnects
export PX4_PARAM_SDLOG_PROFILE=147      # SITL default (131) + high rate: attitude and motors at full rate

cd "$PX4_DIR"
echo "PX4 SITL gz_x500 in world '$WORLD' - MAVLink for fly.py on udp 14540, logs in"
echo "  $PX4_DIR/build/px4_sitl_default/rootfs/log/"
exec env PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD="$WORLD" \
    ./build/px4_sitl_default/bin/px4
