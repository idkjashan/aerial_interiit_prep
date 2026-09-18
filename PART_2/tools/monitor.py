#!/usr/bin/env python3
"""Graphical Flight Telemetry & Rotor Effectiveness HUD Monitor for Part 2.

Displays real-time flight metrics, actuator motor saturation, and control allocator
effectiveness for Part 2 deliverables:
  - Live Altitude and Altitude Loss (m)
  - Current Tilt and Maximum Tilt (deg)
  - Current Yaw Rate and Peak Yaw Rate (deg/s)
  - Actuator Output Gauges (Motors 1 to 4 in rad/s and 0.0 to 1.0)
  - Control Allocator Effectiveness Column Vector B1
  - Real-time Outcome Verdict Banner
"""
import sys
import os
import math
import time
import struct
import cv2
import numpy as np

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

mav = mavutil.mavlink


class Part2Monitor:
    def __init__(self, url=None):
        # Preferred targets: udpout to local PX4 GCS port avoids binding conflicts
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
                conn = mavutil.mavlink_connection(target, source_system=255, source_component=192)
                if target.startswith("udpout"):
                    conn.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                hb = conn.wait_heartbeat(timeout=1.0)
                if hb is not None:
                    self.m = conn
                    print(f"[✓] Part 2 HUD Monitor connected to PX4 SITL on {target}")
                    break
            except Exception:
                pass

        if self.m is None:
            print("Warning: Waiting for PX4 heartbeat on udpout:127.0.0.1:18570...")
            self.m = mavutil.mavlink_connection("udpout:127.0.0.1:18570", source_system=255, source_component=192)

        # Telemetry state
        self.initial_hover_alt = None
        self.current_alt = 0.0
        self.max_tilt = 0.0
        self.max_yaw_rate = 0.0
        self.altitude_loss = 0.0
        self.tilt = 0.0
        self.yaw_rate = 0.0
        self.armed = False
        self.mode = "STANDBY"

        self.scale = 1.0
        self.motor = 1
        self.esc = [0.0, 0.0, 0.0, 0.0]        # Normalized 0.0 to 1.0
        self.esc_rads = [0.0, 0.0, 0.0, 0.0]   # Actual speed 150 to 1000 rad/s

        self.last_hb = 0.0
        self.last_param_check = 0.0
        self.last_stream_req = 0.0

    def request_streams(self):
        tgt_sys = self.m.target_system or 1
        tgt_comp = self.m.target_component or 1
        for msg_id, hz in (
            (mav.MAVLINK_MSG_ID_ATTITUDE, 30),
            (mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
            (mav.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 20),
            (mav.MAVLINK_MSG_ID_HEARTBEAT, 5),
        ):
            self.m.mav.command_long_send(
                tgt_sys, tgt_comp,
                mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, 1e6 / hz, 0, 0, 0, 0, 0
            )

    def update_telemetry(self):
        now = time.monotonic()

        # Send GCS heartbeat every 1 second to keep PX4 streaming
        if now - self.last_hb > 1.0:
            self.m.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_hb = now

        # Request message streams every 5 seconds
        if now - self.last_stream_req > 5.0:
            self.request_streams()
            self.last_stream_req = now

        # Poll CA_EFF_SCALE and CA_EFF_MOTOR every 1 second
        if now - self.last_param_check > 1.0:
            tgt_sys = self.m.target_system or 1
            tgt_comp = self.m.target_component or 1
            self.m.mav.param_request_read_send(tgt_sys, tgt_comp, b"CA_EFF_SCALE", -1)
            self.m.mav.param_request_read_send(tgt_sys, tgt_comp, b"CA_EFF_MOTOR", -1)
            self.last_param_check = now

        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if t == "HEARTBEAT":
                self.armed = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
                custom_mode = int(msg.custom_mode)
                main_mode = (custom_mode >> 16) & 0xFF
                sub_mode = (custom_mode >> 24) & 0xFF
                if main_mode == 1:
                    self.mode = "MANUAL"
                elif main_mode == 2:
                    self.mode = "ALTCTL (Altitude Hold)"
                elif main_mode == 3:
                    self.mode = "POSCTL (Position Hold)"
                elif main_mode == 4:
                    sub_names = {
                        1: "AUTO_READY",
                        2: "AUTO_TAKEOFF",
                        3: "AUTO_LOITER (Hover)",
                        4: "AUTO_MISSION",
                        5: "AUTO_RTL",
                        6: "AUTO_LAND",
                    }
                    self.mode = sub_names.get(sub_mode, f"AUTO_{sub_mode}")
                elif main_mode == 6:
                    self.mode = "OFFBOARD"
                else:
                    self.mode = f"MODE_{main_mode}_{sub_mode}"

            elif t == "LOCAL_POSITION_NED":
                self.current_alt = -msg.z
                if self.initial_hover_alt is None and self.current_alt > 12.0:
                    self.initial_hover_alt = self.current_alt
                if self.initial_hover_alt is not None:
                    loss = max(0.0, self.initial_hover_alt - self.current_alt)
                    self.altitude_loss = max(self.altitude_loss, loss)

            elif t == "ATTITUDE":
                roll = abs(msg.roll * 57.2958)
                pitch = abs(msg.pitch * 57.2958)
                self.tilt = math.hypot(roll, pitch)
                self.yaw_rate = abs(msg.yawspeed * 57.2958)
                if self.initial_hover_alt is not None:
                    self.max_tilt = max(self.max_tilt, self.tilt)
                    self.max_yaw_rate = max(self.max_yaw_rate, self.yaw_rate)

            elif t == "SERVO_OUTPUT_RAW":
                # Gazebo ESC speed in rad/s: 0 = stopped, 150 = idle, 1000 = max thrust
                for i in range(4):
                    raw_val = float(getattr(msg, f"servo{i+1}_raw", 0))
                    self.esc_rads[i] = raw_val
                    if raw_val <= 0:
                        self.esc[i] = 0.0
                    elif raw_val < 150.0:
                        self.esc[i] = 0.0
                    else:
                        # Normalized 0..1 command = (rads - 150) / (1000 - 150)
                        self.esc[i] = max(0.0, min(1.0, (raw_val - 150.0) / 850.0))

            elif t == "PARAM_VALUE":
                p_id = msg.param_id.strip("\x00")
                if p_id == "CA_EFF_SCALE":
                    new_scale = float(msg.param_value)
                    if abs(new_scale - self.scale) > 0.01:
                        self.scale = new_scale
                        self.max_tilt = self.tilt
                        self.max_yaw_rate = self.yaw_rate
                        self.altitude_loss = 0.0
                        self.initial_hover_alt = self.current_alt
                elif p_id == "CA_EFF_MOTOR":
                    if msg.param_type == mav.MAV_PARAM_TYPE_INT32:
                        self.motor = struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
                    else:
                        self.motor = int(msg.param_value)

    def render(self):
        canvas = np.zeros((620, 1000, 3), dtype=np.uint8)

        # 1. Header Banner
        pct = int(round(self.scale * 100))
        if pct >= 75:
            header_col = (0, 140, 0)      # Green
            verdict = "HOLDS HOVER (Stable, 0m Alt Loss)"
        elif pct >= 50:
            header_col = (0, 140, 255)    # Orange
            verdict = "HOLDS HOVER (Compensating after transient)"
        elif pct > 0:
            header_col = (0, 0, 200)      # Red
            verdict = "ALTITUDE LOSS (Descends upright to ground)"
        else:
            header_col = (0, 0, 150)      # Dark Red
            verdict = "UNCONTROLLABLE (Tumbles, ground reached)"

        cv2.rectangle(canvas, (0, 0), (1000, 65), header_col, -1)
        cv2.putText(canvas, f"PART 2: ROTOR EFFECTIVENESS = {pct}% (MOTOR {self.motor})",
                    (25, 42), cv2.FONT_HERSHEY_DUPLEX, 0.85, (255, 255, 255), 2)
        cv2.putText(canvas, f"OUTCOME: {verdict}", (520, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        # 2. Main Metrics Grid
        cv2.rectangle(canvas, (20, 80), (980, 240), (25, 25, 30), -1)
        cv2.rectangle(canvas, (20, 80), (980, 240), (60, 60, 70), 1)

        def metric(title, value, unit, x, y, col=(0, 255, 255)):
            cv2.putText(canvas, title, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (160, 160, 160), 1)
            cv2.putText(canvas, f"{value} {unit}", (x, y + 28), cv2.FONT_HERSHEY_DUPLEX, 0.78, col, 2)

        # Col 1: Altitude
        loss_col = (0, 255, 0) if self.altitude_loss < 1.0 else ((0, 165, 255) if self.altitude_loss < 5.0 else (0, 0, 255))
        metric("CURRENT ALTITUDE", f"{self.current_alt:.2f}", "m", 40, 115)
        metric("ALTITUDE LOSS", f"{self.altitude_loss:.2f}", "m", 40, 185, loss_col)

        # Col 2: Tilt
        tilt_col = (0, 255, 0) if self.max_tilt < 10.0 else ((0, 165, 255) if self.max_tilt < 30.0 else (0, 0, 255))
        metric("CURRENT TILT", f"{self.tilt:.1f}", "deg", 280, 115)
        metric("MAX TILT", f"{self.max_tilt:.1f}", "deg", 280, 185, tilt_col)

        # Col 3: Yaw Rate
        yaw_col = (0, 255, 0) if self.max_yaw_rate < 15.0 else ((0, 165, 255) if self.max_yaw_rate < 100.0 else (0, 0, 255))
        metric("CURRENT YAW RATE", f"{self.yaw_rate:.1f}", "deg/s", 520, 115)
        metric("MAX YAW RATE", f"{self.max_yaw_rate:.1f}", "deg/s", 520, 185, yaw_col)

        # Col 4: Motor 1 at End & Status
        m1_col = (0, 255, 0) if self.esc[0] > 0.4 else (0, 0, 255)
        metric("MOTOR 1 (ESC 1)", f"{self.esc[0]:.2f}", f"[{self.esc_rads[0]:.0f} rad/s]", 750, 115, m1_col)
        metric("STATUS / MODE", self.mode, f"({'ARMED' if self.armed else 'DISARMED'})", 750, 185, (0, 255, 255))

        # 3. Actuator Output Gauges (ESCs 1-4)
        cv2.rectangle(canvas, (20, 260), (480, 580), (25, 25, 30), -1)
        cv2.rectangle(canvas, (20, 260), (480, 580), (60, 60, 70), 1)
        cv2.putText(canvas, "ACTUATOR OUTPUTS (0.0 to 1.0)", (40, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        for i in range(4):
            y_base = 345 + i * 55
            val = self.esc[i]
            rads = self.esc_rads[i]
            label = f"Motor {i+1} (CCW)" if i in (0, 1) else f"Motor {i+1} (CW)"
            if i == 0:
                label += f" [Effectiveness x{self.scale:.2f}]"
            cv2.putText(canvas, label, (40, y_base - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 180, 180), 1)
            # Background bar
            cv2.rectangle(canvas, (40, y_base), (370, y_base + 22), (40, 40, 50), -1)
            # Filled bar
            bar_w = int(330 * val)
            bar_col = (0, 200, 255) if i == 0 else (0, 255, 100)
            if val >= 0.95 or rads >= 950:
                bar_col = (0, 0, 255)  # Saturated Red
            cv2.rectangle(canvas, (40, y_base), (40 + bar_w, y_base + 22), bar_col, -1)
            cv2.putText(canvas, f"{val:.2f} ({rads:.0f} rad/s)", (380, y_base + 17), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1)

        # 4. Effectiveness Matrix B Column Vector
        cv2.rectangle(canvas, (500, 260), (980, 580), (25, 25, 30), -1)
        cv2.rectangle(canvas, (500, 260), (980, 580), (60, 60, 70), 1)
        cv2.putText(canvas, f"CONTROL ALLOCATOR COLUMN B (MOTOR {self.motor})", (520, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)

        # Physical constants for x500 Motor 1: roll=-1.43, pitch=0.845, yaw=0.325, thrust=-6.5
        b_roll = -1.43 * self.scale
        b_pitch = 0.845 * self.scale
        b_yaw = 0.325 * self.scale
        b_thrust = -6.5 * self.scale

        rows = [
            ("Roll Torque (L_roll):", f"{b_roll:+.3f}", "Nm / cmd"),
            ("Pitch Torque (M_pitch):", f"{b_pitch:+.3f}", "Nm / cmd"),
            ("Yaw Torque (N_yaw):", f"{b_yaw:+.3f}", "Nm / cmd"),
            ("Collective Thrust (Z_thrust):", f"{b_thrust:+.3f}", "N / cmd"),
        ]

        for i, (name, val_s, u) in enumerate(rows):
            y_pos = 350 + i * 50
            cv2.putText(canvas, name, (520, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)
            cv2.putText(canvas, f"{val_s}  {u}", (810, y_pos), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 255), 1)

        cv2.putText(canvas, "Press 'q' in this window to close monitor.", (520, 555), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (140, 140, 140), 1)

        cv2.imshow("Part 2 - Rotor Effectiveness HUD Monitor", canvas)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return False
        return True


def main():
    monitor = Part2Monitor()
    try:
        while True:
            monitor.update_telemetry()
            if not monitor.render():
                break
            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
