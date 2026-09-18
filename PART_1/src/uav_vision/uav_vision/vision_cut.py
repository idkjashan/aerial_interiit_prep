#!/usr/bin/env python3
"""Vision Disconnect & Reconnect Tool.

Allows demonstrating vision sensor loss and recovery on demand:
  - disconnect : pauses the camera bridge, cutting image frames to the VO node.
  - reconnect  : resumes the camera bridge, restoring frames and allowing VO recovery.
  - test [sec] : disconnects vision for [sec] seconds (default: 5.0) and automatically reconnects.

Usage:
  ros2 run uav_vision vision_cut disconnect
  ros2 run uav_vision vision_cut reconnect
  ros2 run uav_vision vision_cut test 5
"""
import sys
import time
import subprocess


def find_bridge_pids():
    """Find PID of parameter_bridge handling camera topics."""
    try:
        out = subprocess.check_output(["pgrep", "-f", "parameter_bridge.*IMX214"], text=True)
        return [int(p.strip()) for p in out.splitlines() if p.strip()]
    except subprocess.CalledProcessError:
        return []


def disconnect():
    pids = find_bridge_pids()
    if not pids:
        print("[WARN] No active camera bridge found matching 'parameter_bridge.*IMX214'.")
        print("       Is the simulation running?")
        return False
    for pid in pids:
        subprocess.run(["kill", "-STOP", str(pid)])
    print("=" * 60)
    print(f"[ACTION] VISION DISCONNECTED! (Paused PID(s): {pids})")
    print("         - Camera frames stopped.")
    print("         - Visual odometry health will transition: OK -> DEGRADED -> LOST.")
    print("         - PX4 will enter failsafe / degraded altitude hold.")
    print("=" * 60)
    return True


def reconnect():
    pids = find_bridge_pids()
    if not pids:
        print("[WARN] No active camera bridge found matching 'parameter_bridge.*IMX214'.")
        return False
    for pid in pids:
        subprocess.run(["kill", "-CONT", str(pid)])
    print("=" * 60)
    print(f"[ACTION] VISION RECONNECTED! (Resumed PID(s): {pids})")
    print("         - Camera frames restored.")
    print("         - Visual odometry health will transition: LOST -> OK.")
    print("         - PX4 EKF2 fuses vision and returns to position hold.")
    print("=" * 60)
    return True


def test(seconds=5.0):
    if not disconnect():
        return
    print(f"\nWaiting {seconds:.1f} seconds to simulate sensor dropout...")
    time.sleep(seconds)
    reconnect()
    print("\n[SUCCESS] Vision failure & recovery cycle completed.")


def main():
    cmd = sys.argv[1].lower() if len(sys.argv) > 1 else "help"
    if cmd in ("disconnect", "cut", "stop", "off"):
        disconnect()
    elif cmd in ("reconnect", "resume", "cont", "on"):
        reconnect()
    elif cmd == "test":
        dur = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
        test(dur)
    else:
        print("Usage:")
        print("  ros2 run uav_vision vision_cut disconnect   # Cut vision stream")
        print("  ros2 run uav_vision vision_cut reconnect    # Resume vision stream")
        print("  ros2 run uav_vision vision_cut test [sec]   # Cut for [sec] seconds and restore")


if __name__ == "__main__":
    main()
