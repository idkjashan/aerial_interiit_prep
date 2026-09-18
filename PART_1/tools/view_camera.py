#!/usr/bin/env python3
"""Live Camera Viewer with Flight Telemetry HUD.

Subscribes to:
  - /uav/rgb (camera feed)
  - /uav_visual_odometry/health
  - /uav_visual_odometry/quality
  - /fmu/out/vehicle_local_position
  - /fmu/out/vehicle_status

Usage:
  source /opt/ros/humble/setup.bash
  source ~/px4_ros_ws/install/setup.bash
  export ROS_DOMAIN_ID=77
  python3 PART_1/tools/view_camera.py
"""
import sys
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus


def px4_sub_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class CameraViewer(Node):
    def __init__(self):
        super().__init__("uav_camera_viewer")

        self.sub_img = self.create_subscription(Image, "/uav/rgb", self._on_img, 10)
        self.sub_health = self.create_subscription(String, "/uav_visual_odometry/health", self._on_health, 10)
        self.sub_quality = self.create_subscription(Int32, "/uav_visual_odometry/quality", self._on_quality, 10)
        self.sub_lpos = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._on_lpos, px4_sub_qos())
        self.sub_lpos_v1 = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._on_lpos, px4_sub_qos())
        self.sub_status = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self._on_status, px4_sub_qos())
        self.sub_status_v1 = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, px4_sub_qos())

        self.health = "UNKNOWN"
        self.quality = 0
        self.lpos = None
        self.status = None
        self.frame_count = 0
        self.fps = 0.0
        self.last_time = self.get_clock().now()

        self.get_logger().info("Camera viewer up. Waiting for images on /uav/rgb...")

    def _on_health(self, msg):
        self.health = msg.data

    def _on_quality(self, msg):
        self.quality = msg.data

    def _on_lpos(self, msg):
        self.lpos = msg

    def _on_status(self, msg):
        self.status = msg

    def _on_img(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.frame_count += 1
        if dt >= 1.0:
            self.fps = self.frame_count / dt
            self.frame_count = 0
            self.last_time = now

        # Convert ROS Image (rgb8) to OpenCV BGR image
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().warn(f"Failed to decode image: {e}")
            return

        # Resize for display convenience (e.g. 960x540)
        h, w = frame.shape[:2]
        disp = cv2.resize(frame, (960, 540))

        # HUD Overlay
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED if self.status else False
        nav = self.status.nav_state if self.status else "UNKNOWN"
        alt = f"{-self.lpos.z:.2f} m" if self.lpos else "n/a"
        x = f"{self.lpos.x:.2f}" if self.lpos else "0.0"
        y = f"{self.lpos.y:.2f}" if self.lpos else "0.0"

        # Color based on health
        color = (0, 255, 0) if self.health == "OK" else ((0, 165, 255) if self.health == "DEGRADED" else (0, 0, 255))

        # Top banner
        cv2.rectangle(disp, (0, 0), (960, 45), (20, 20, 20), -1)
        cv2.putText(disp, f"CAM: 1920x1080 @ {self.fps:.1f} FPS", (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(disp, f"HEALTH: {self.health} ({self.quality}%)", (320, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(disp, f"ALT: {alt} | POS: [{x}, {y}]", (600, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

        # Crosshair in center
        cx, cy = 480, 270
        cv2.drawMarker(disp, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)

        cv2.imshow("UAV Downward Camera (/uav/rgb) - Press 'q' to exit", disp)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CameraViewer()
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
