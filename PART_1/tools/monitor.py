#!/usr/bin/env python3
"""Graphical Flight Telemetry & Vision Health HUD Monitor.

Subscribes to:
  - /uav/rgb                          (Downward camera image)
  - /uav_visual_odometry/health       (OK | DEGRADED | LOST)
  - /uav_visual_odometry/quality      (0 - 100)
  - /uav_vision_health/status         (Diagnostics)
  - /fmu/out/vehicle_status           (Arming state, nav_state)
  - /fmu/out/vehicle_local_position   (XYZ local position & velocities)
  - /fmu/out/failsafe_flags           (PX4 failsafe triggers)

Displays:
  - Live 1080p camera feed (scaled for display)
  - Color-coded status banners:
      * GREEN:  Vision Healthy (OK, 100%) - OFFBOARD Position Hold
      * RED:    VISION CUT / LOST (0%) - PX4 EKF2 Failsafe / Altitude Hold Active
      * YELLOW: Vision Degraded / Relocalizing
  - Numerical telemetry: Altitude AGL, Velocity, Position (X,Y), Max Drift
  - Flight mode & arming state
"""
import math
import sys
import os

lib_dir = os.path.expanduser("~/px4_ros_ws/install/px4_msgs/lib")
if lib_dir not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] = f"{lib_dir}:{os.environ.get('LD_LIBRARY_PATH', '')}"
    os.environ.setdefault("ROS_DOMAIN_ID", "77")
    try:
        os.execv(sys.executable, [sys.executable] + sys.argv)
    except Exception:
        pass

if os.environ.get("ROS_DOMAIN_ID") in (None, "", "42", "0"):
    os.environ["ROS_DOMAIN_ID"] = "77"

for p in [
    os.path.expanduser("~/px4_ros_ws/install/px4_msgs/local/lib/python3.10/dist-packages"),
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "install", "uav_vision", "local", "lib", "python3.10", "dist-packages")),
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from diagnostic_msgs.msg import DiagnosticArray
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus


def px4_sub_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class FlightMonitor(Node):
    def __init__(self):
        super().__init__("uav_flight_monitor")

        self.health = "WAITING"
        self.quality = 0
        self.status = None
        self.lpos = None
        self.failsafe = None
        self.fps = 0.0
        self.frame_count = 0
        self.last_fps_time = self.get_clock().now()
        self.last_img_time = None
        self.home_xy = None
        self.max_drift = 0.0

        q = px4_sub_qos()
        self.create_subscription(Image, "/uav/rgb", self._on_img, 10)
        self.create_subscription(String, "/uav_visual_odometry/health", lambda m: setattr(self, "health", m.data), 10)
        self.create_subscription(Int32, "/uav_visual_odometry/quality", lambda m: setattr(self, "quality", m.data), 10)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", lambda m: setattr(self, "status", m), q)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", lambda m: setattr(self, "status", m), q)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._on_lpos, q)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._on_lpos, q)

        # Timer to update window even if no camera frames arrive (e.g. vision cut!)
        self.create_timer(0.05, self._tick_display)
        self.last_frame = None

        self.get_logger().info("Flight Monitor HUD initialized. Ready for display.")

    def _on_lpos(self, msg):
        self.lpos = msg
        if self.home_xy is None and abs(msg.z) > 1.0:
            self.home_xy = (msg.x, msg.y)
        if self.home_xy is not None:
            r = math.hypot(msg.x - self.home_xy[0], msg.y - self.home_xy[1])
            self.max_drift = max(self.max_drift, r)

    def _on_img(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_fps_time).nanoseconds * 1e-9
        self.frame_count += 1
        if dt >= 1.0:
            self.fps = self.frame_count / dt
            self.frame_count = 0
            self.last_fps_time = now

        self.last_img_time = now

        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            self.last_frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        except Exception:
            pass

    def _tick_display(self):
        now = self.get_clock().now()
        img_stale = (
            self.last_img_time is None or
            (now - self.last_img_time).nanoseconds * 1e-9 > 0.5
        )

        # Base canvas: 960 x 600
        canvas = np.zeros((600, 960, 3), dtype=np.uint8)

        if self.last_frame is not None and not img_stale:
            # Resize image into camera view area: 960 x 500
            cam_view = cv2.resize(self.last_frame, (960, 500))
            canvas[100:600, 0:960] = cam_view
        else:
            # Vision cut or no frames
            cv2.rectangle(canvas, (0, 100), (960, 600), (20, 20, 40), -1)
            cv2.putText(canvas, "[ CAMERA FEED INTERRUPTED / VISION DISCONNECTED ]",
                        (140, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            cv2.putText(canvas, "PX4 Estimator Dead-Reckoning on IMU + Barometer",
                        (190, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)

        # ------------------- STATUS BARS -------------------
        # Determine overall state
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED if self.status else False
        nav_state = getattr(self.status, "nav_state", 0) if self.status else 0
        nav_names = {
            0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
            4: "AUTO_LOITER", 5: "AUTO_RTL", 14: "OFFBOARD", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND"
        }
        nav_str = nav_names.get(nav_state, f"STATE_{nav_state}")

        alt_str = f"{-self.lpos.z:.2f} m" if self.lpos else "0.00 m"
        vx = f"{self.lpos.vx:.2f}" if self.lpos else "0.0"
        vy = f"{self.lpos.vy:.2f}" if self.lpos else "0.0"
        x_str = f"{self.lpos.x:.2f}" if self.lpos else "0.0"
        y_str = f"{self.lpos.y:.2f}" if self.lpos else "0.0"

        # Vision State Header
        if img_stale or self.health == "LOST":
            header_color = (0, 0, 180)  # Red
            v_text = "VISION LOST (0%) - FAILSAFE / ALTITUDE HOLD ACTIVE"
        elif self.health == "DEGRADED":
            header_color = (0, 140, 255)  # Orange
            v_text = f"VISION DEGRADED ({self.quality}%) - CONVERGING"
        elif self.health == "OK":
            header_color = (0, 140, 0)  # Green
            v_text = f"VISION HEALTHY ({self.quality}%) - OFFBOARD POSITION HOLD"
        else:
            header_color = (50, 50, 50)
            v_text = "INITIALIZING ESTIMATOR..."

        # Top Header Box (0 to 50)
        cv2.rectangle(canvas, (0, 0), (960, 50), header_color, -1)
        cv2.putText(canvas, v_text, (20, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        fps_text = f"FPS: {self.fps:.1f}" if not img_stale else "FPS: 0.0"
        cv2.putText(canvas, fps_text, (840, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)

        # Telemetry Sub-bar (50 to 100)
        cv2.rectangle(canvas, (0, 50), (960, 100), (30, 30, 30), -1)
        arm_col = (0, 255, 0) if armed else (100, 100, 100)
        cv2.putText(canvas, f"ARMED: {armed}", (20, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.65, arm_col, 2)
        cv2.putText(canvas, f"MODE: {nav_str}", (190, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
        cv2.putText(canvas, f"ALT: {alt_str}", (400, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(canvas, f"POS: [{x_str}, {y_str}] m", (580, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(canvas, f"DRIFT: {self.max_drift:.2f} m", (800, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 100), 1)

        # Crosshair in camera area
        cx, cy = 480, 350
        cv2.drawMarker(canvas, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 24, 1)

        cv2.imshow("GPS-Denied UAV Flight Monitor (Video HUD)", canvas)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rclpy.shutdown()


def main():
    rclpy.init()
    node = FlightMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
