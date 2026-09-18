#!/usr/bin/env bash
# ==============================================================================
# vision_cut.sh: Disconnect and Reconnect Camera Stream to Test Failsafe & Recovery
# ==============================================================================
# Usage:
#   ./tools/vision_cut.sh disconnect     # Cuts the camera stream (pauses bridge)
#   ./tools/vision_cut.sh reconnect      # Restores camera stream (resumes bridge)
#   ./tools/vision_cut.sh test [sec]     # Cuts for [sec] seconds (default 5) and restores
# ==============================================================================

ACTION="${1:-help}"
DURATION="${2:-5}"

get_pids() {
  pgrep -f "parameter_bridge.*IMX214" || true
}

disconnect() {
  PIDS=$(get_pids)
  if [ -z "$PIDS" ]; then
    echo "[WARN] No active camera bridge found (is simulation running?)."
    exit 1
  fi
  for pid in $PIDS; do
    kill -STOP "$pid"
  done
  echo "================================================================"
  echo " [!] VISION STREAM DISCONNECTED (Paused PIDs: $PIDS)"
  echo "     - Camera frames stopped arriving at Visual Odometry node."
  echo "     - Monitor HUD will turn RED (VISION LOST)."
  echo "     - PX4 EKF2 enters failsafe (Altitude Mode hold)."
  echo "     - Drone will hold 10m altitude on Barometer without landing."
  echo "================================================================"
}

reconnect() {
  PIDS=$(get_pids)
  if [ -z "$PIDS" ]; then
    echo "[WARN] No camera bridge found."
    exit 1
  fi
  for pid in $PIDS; do
    kill -CONT "$pid"
  done
  echo "================================================================"
  echo " [✓] VISION STREAM RECONNECTED (Resumed PIDs: $PIDS)"
  echo "     - Camera frames restored."
  echo "     - Visual Odometry re-acquires features (Health -> OK)."
  echo "     - Monitor HUD turns GREEN (VISION HEALTHY)."
  echo "     - Drone locks back into rock-solid position hold."
  echo "================================================================"
}

case "$ACTION" in
  disconnect|cut|stop|off)
    disconnect
    ;;
  reconnect|restore|resume|cont|on)
    reconnect
    ;;
  test)
    disconnect
    echo
    echo "Waiting $DURATION seconds to demonstrate failsafe behavior..."
    for i in $(seq "$DURATION" -1 1); do
      echo "  ... restoring in $i s"
      sleep 1
    done
    echo
    reconnect
    ;;
  *)
    echo "Usage:"
    echo "  ./tools/vision_cut.sh disconnect     # Cut camera stream"
    echo "  ./tools/vision_cut.sh reconnect      # Restore camera stream"
    echo "  ./tools/vision_cut.sh test [sec]     # Test 5-second failure & recovery"
    ;;
esac
