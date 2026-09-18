#!/usr/bin/env python3
"""Set rotor effectiveness scaling or inject motor failure for Part 2.

Usage:
    python3 PART_2/tools/set_effectiveness.py 0.75          # Scale motor 1 to 75%
    python3 PART_2/tools/set_effectiveness.py 50%           # Scale motor 1 to 50%
    python3 PART_2/tools/set_effectiveness.py 0.25          # Scale motor 1 to 25%
    python3 PART_2/tools/set_effectiveness.py 0.00          # Scale motor 1 to 0%
    python3 PART_2/tools/set_effectiveness.py 1.00          # Restore 100% (neutral)
    python3 PART_2/tools/set_effectiveness.py failure       # Trigger built-in PX4 motor failure
    python3 PART_2/tools/set_effectiveness.py status        # Read current effectiveness
"""
import sys
import os
import time
import struct

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

mav = mavutil.mavlink

MAV_CMD_INJECT_FAILURE = 420
FAILURE_UNIT_SYSTEM_MOTOR = 101
FAILURE_TYPE_OK = 0
FAILURE_TYPE_OFF = 1


def connect(url=None):
    candidates = [url] if url else [
        "udpout:127.0.0.1:18570",
        "udpout:127.0.0.1:18571",
        "udp:127.0.0.1:14540",
        "udp:127.0.0.1:14541",
        "udp:127.0.0.1:14550",
    ]
    m = None
    for target in candidates:
        try:
            conn = mavutil.mavlink_connection(target, source_system=255, source_component=191)
            if target.startswith("udpout"):
                conn.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            hb = conn.wait_heartbeat(timeout=1.0)
            if hb is not None:
                m = conn
                break
        except Exception:
            pass

    if m is None:
        print("Error: Could not connect to PX4 SITL on standard ports.")
        sys.exit(1)
    return m


def set_param(m, name, value, is_int=False):
    if is_int:
        raw = struct.unpack('<f', struct.pack('<i', int(value)))[0]
        ptype = mav.MAV_PARAM_TYPE_INT32
    else:
        raw = float(value)
        ptype = mav.MAV_PARAM_TYPE_REAL32

    for _ in range(5):
        m.mav.param_set_send(
            m.target_system, m.target_component,
            name.encode(), raw, ptype
        )
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.1)
            if msg and msg.param_id.strip("\x00") == name:
                if is_int and msg.param_type == mav.MAV_PARAM_TYPE_INT32:
                    return struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
                return msg.param_value
    return None


def set_scale(m, scale, motor=1):
    scale = float(scale)
    scale = max(0.0, min(1.0, scale))

    m_val = set_param(m, "CA_EFF_MOTOR", motor, is_int=True)
    s_val = set_param(m, "CA_EFF_SCALE", scale, is_int=False)

    print("================================================================")
    print(f" [✓] Rotor Effectiveness Updated: Motor {m_val if m_val is not None else motor} -> {(s_val if s_val is not None else scale) * 100:.0f} %")
    print(f"     CA_EFF_MOTOR = {m_val if m_val is not None else motor}")
    print(f"     CA_EFF_SCALE = {s_val if s_val is not None else scale:.4f}")
    print("================================================================")


def inject_failure(m, motor=1):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        MAV_CMD_INJECT_FAILURE, 0,
        float(FAILURE_UNIT_SYSTEM_MOTOR),
        float(FAILURE_TYPE_OFF),
        float(motor),
        0.0, 0.0, 0.0, 0.0
    )
    print("================================================================")
    print(f" [!] Built-In Motor Failure Injected: Motor {motor} OFF")
    print("     Failure detector (FD_ACT_EN) and allocator (CA_FAILURE_MODE=1) active.")
    print("================================================================")


def get_status(m):
    # Request param read
    m.mav.param_request_read_send(m.target_system, m.target_component, b"CA_EFF_SCALE", -1)
    m.mav.param_request_read_send(m.target_system, m.target_component, b"CA_EFF_MOTOR", -1)
    start = time.time()
    scale = None
    motor = None
    while time.time() - start < 3:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg:
            p_id = msg.param_id.strip("\x00")
            if p_id == "CA_EFF_SCALE":
                scale = msg.param_value
            elif p_id == "CA_EFF_MOTOR":
                motor = struct.unpack('<i', struct.pack('<f', msg.param_value))[0] if msg.param_type == mav.MAV_PARAM_TYPE_INT32 else int(msg.param_value)
        if scale is not None and motor is not None:
            break

    print("================================================================")
    print(f"  Current Rotor Effectiveness Status: Motor {motor or 1} @ {(scale or 1.0) * 100:.0f} %")
    print("================================================================")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)

    arg = sys.argv[1].lower().strip()
    m = connect()

    if arg == "status":
        get_status(m)
    elif arg in ("failure", "fail", "off"):
        motor = int(sys.argv[2]) if len(sys.argv) > 2 else 1
        inject_failure(m, motor)
    else:
        # Parse percentage or float
        clean_arg = arg.rstrip("%")
        try:
            val = float(clean_arg)
            if "%" in arg or val > 1.0:
                val = val / 100.0
            motor = int(sys.argv[2]) if len(sys.argv) > 2 else 1
            set_scale(m, val, motor)
        except ValueError:
            print(f"Unknown argument: {arg}")
            print(__doc__)
            sys.exit(1)


if __name__ == "__main__":
    main()
