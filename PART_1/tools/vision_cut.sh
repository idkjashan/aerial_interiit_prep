#!/usr/bin/env bash
# Cut and restore the camera stream to test vision loss and recovery.
# It pauses (SIGSTOP) / resumes (SIGCONT) the ros_gz_bridge process carrying the camera.
#   ./vision_cut.sh disconnect
#   ./vision_cut.sh reconnect
#   ./vision_cut.sh test [sec]     # cut for sec seconds (default 5), then restore

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
  echo "camera stream paused (bridge PIDs: $PIDS) - VO health should go LOST"
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
  echo "camera stream resumed (bridge PIDs: $PIDS) - VO health should return to OK"
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
    echo "restoring in $DURATION s..."
    for i in $(seq "$DURATION" -1 1); do
      echo "  ... restoring in $i s"
      sleep 1
    done
    echo
    reconnect
    ;;
  *)
    echo "usage: $0 disconnect | reconnect | test [sec]"
    ;;
esac
