#!/usr/bin/env python3
"""Manual flight control utility for Part 2 (over MAVLink).

Usage:
    python3 PART_2/tools/manual_control.py status
    python3 PART_2/tools/manual_control.py arm
    python3 PART_2/tools/manual_control.py takeoff [altitude_m]   # default 20.0 m
    python3 PART_2/tools/manual_control.py hover
    python3 PART_2/tools/manual_control.py land
    python3 PART_2/tools/manual_control.py disarm
"""
import sys
import os
import time
import math

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

mav = mavutil.mavlink


class FlightController:
    def __init__(self, url=None):
        candidates = [url] if url else [
            "udpout:127.0.0.1:18570",
            "udpout:127.0.0.1:18571",
            "udp:127.0.0.1:14540",
            "udp:127.0.0.1:14541",
            "udp:127.0.0.1:14550",
        ]
        self.m = None
        for target in candidates:
            try:
                conn = mavutil.mavlink_connection(target, source_system=255, source_component=190)
                if target.startswith("udpout"):
                    conn.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                hb = conn.wait_heartbeat(timeout=1.0)
                if hb is not None:
                    self.m = conn
                    break
            except Exception:
                pass

        if self.m is None:
            print("Error: No heartbeat from PX4. Is SITL running? (run ./tools/launch_sim.sh)")
            sys.exit(1)

    def cmd(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        self.m.mav.command_long_send(
            self.m.target_system,
            self.m.target_component,
            command,
            0,
            float(p1), float(p2), float(p3), float(p4), float(p5), float(p6), float(p7)
        )

    def arm(self):
        print("Waiting for estimator & arming vehicle...")
        for _ in range(15):
            self.cmd(mav.MAV_CMD_COMPONENT_ARM_DISARM, p1=1.0)
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if hb and (hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED):
                print("Armed successfully!")
                return True
            time.sleep(1.0)
        print("Warning: Arming attempt complete, verify status.")
        return False

    def disarm(self, force=True):
        print("Sending DISARM command...")
        p2 = 21196.0 if force else 0.0
        self.cmd(mav.MAV_CMD_COMPONENT_ARM_DISARM, p1=0.0, p2=p2)

    def takeoff(self, alt=20.0):
        # Set MIS_TAKEOFF_ALT param
        self.m.mav.param_set_send(
            self.m.target_system, self.m.target_component,
            b"MIS_TAKEOFF_ALT", float(alt), mav.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(0.1)

        print(f"Arming and initiating takeoff to {alt:.1f} m...")
        self.arm()
        time.sleep(0.5)

        # Send MAV_CMD_NAV_TAKEOFF
        self.cmd(mav.MAV_CMD_NAV_TAKEOFF, p1=0.0, p2=0.0, p3=0.0, p4=math.nan, p5=math.nan, p6=math.nan, p7=float(alt))
        print("Climbing to target altitude. Monitoring climb...")
        start = time.time()
        while time.time() - start < 45:
            msg = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
            if msg:
                cur_alt = -msg.z
                print(f"\rCurrent Altitude: {cur_alt:.2f} m / {alt:.1f} m", end="", flush=True)
                if abs(cur_alt - alt) < 0.5:
                    print(f"\nReached target altitude {cur_alt:.2f} m. Stable hover active.")
                    return True
            time.sleep(0.1)
        print("\nClimb monitoring timed out.")

    def hover(self):
        print("Switching to HOLD / LOITER mode...")
        # Custom mode 4 = AUTO_LOITER in PX4
        self.cmd(mav.MAV_CMD_DO_SET_MODE, p1=1.0, p2=4.0)

    def land(self):
        print("Commanding LAND...")
        self.cmd(mav.MAV_CMD_NAV_LAND)

    def get_status(self):
        hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
        armed = bool(hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED) if hb else False
        pos = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2.0)
        alt = f"{-pos.z:.2f} m" if pos else "n/a"
        x = f"{pos.x:.2f}" if pos else "n/a"
        y = f"{pos.y:.2f}" if pos else "n/a"
        att = self.m.recv_match(type="ATTITUDE", blocking=True, timeout=2.0)
        roll = f"{att.roll * 57.2958:.1f} deg" if att else "n/a"
        pitch = f"{att.pitch * 57.2958:.1f} deg" if att else "n/a"
        yaw_rate = f"{att.yawspeed * 57.2958:.1f} deg/s" if att else "n/a"
        return f"Armed: {armed} | Alt: {alt} | Pos: [{x}, {y}] | Roll: {roll} | Pitch: {pitch} | YawRate: {yaw_rate}"


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)

    cmd = sys.argv[1].lower().strip("-")
    fc = FlightController()

    if cmd == "status":
        print(fc.get_status())
    elif cmd == "arm":
        fc.arm()
    elif cmd == "disarm":
        fc.disarm()
    elif cmd == "takeoff":
        alt = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
        fc.takeoff(alt)
    elif cmd == "hover":
        fc.hover()
    elif cmd == "land":
        fc.land()
    else:
        print(f"Unknown command: {cmd}")
        print("Available: status, arm, takeoff, hover, land, disarm")


if __name__ == "__main__":
    main()
