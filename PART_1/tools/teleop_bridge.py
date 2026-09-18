#!/usr/bin/env python3
"""Teleoperation bridge: maps /cmd_vel (from teleop_twist_keyboard) to PX4 offboard velocity setpoints.

Usage:
    Terminal 1:
        ros2 run teleop_twist_keyboard teleop_twist_keyboard

    Terminal 2:
        python3 PART_1/tools/teleop_bridge.py

Controls (standard teleop_twist_keyboard):
    i / , : forward / backward
    j / l : strafe left / right
    u / o : yaw left / right
    t / b : ascend / descend
    k     : stop / zero velocity
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleAttitude, VehicleCommand, VehicleStatus, VehicleLocalPosition


def px4_pub_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def px4_sub_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class TeleopBridge(Node):
    def __init__(self):
        super().__init__("teleop_bridge")

        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.sub_att = self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self._on_att, px4_sub_qos())
        self.sub_att_v1 = self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude_v1", self._on_att, px4_sub_qos())
        self.sub_status = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self._on_status, px4_sub_qos())
        self.sub_status_v1 = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, px4_sub_qos())
        self.sub_lpos = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._on_lpos, px4_sub_qos())
        self.sub_lpos_v1 = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._on_lpos, px4_sub_qos())

        self.pub_ocm = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", px4_pub_qos())
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", px4_pub_qos())
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", px4_pub_qos())

        self.yaw = 0.0
        self.status = None
        self.lpos = None
        self.cmd_linear = [0.0, 0.0, 0.0]  # FLU: [forward, left, up]
        self.cmd_yaw_rate = 0.0            # CCW positive
        self.last_cmd_time = None
        self.takeoff_done = False
        self.hold_origin = None

        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info("Teleop bridge up. Listening to /cmd_vel -> streaming to /fmu/in/trajectory_setpoint at 20 Hz.")

    def _on_status(self, msg):
        self.status = msg

    def _on_lpos(self, msg):
        self.lpos = msg

    def _cmd(self, command, **params):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = command
        for i in range(1, 8):
            setattr(m, f"param{i}", float(params.get(f"param{i}", 0.0)))
        target_sys = int(getattr(self.status, "system_id", 0)) if self.status else 0
        m.target_system = target_sys if target_sys > 0 else 1
        m.target_component = 1
        m.source_system = m.target_system
        m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def _on_att(self, msg):
        q = msg.q  # w, x, y, z in NED/FRD
        # yaw in NED (North=0, CW positive)
        self.yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2))

    def _on_cmd_vel(self, msg):
        self.cmd_linear = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.cmd_yaw_rate = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()
        # If no key pressed within 0.6 s, brake to 0 m/s
        if self.last_cmd_time is None or (now - self.last_cmd_time).nanoseconds * 1e-9 > 0.6:
            vx_body, vy_body, vz_body = 0.0, 0.0, 0.0
            r_yaw = 0.0
        else:
            vx_body, vy_body, vz_body = self.cmd_linear
            r_yaw = self.cmd_yaw_rate

        # Transform body FLU velocities to NED world frame using current heading
        # vx_body = forward (+X FRD), vy_body = left (-Y FRD), vz_body = up (-Z NED)
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)

        vx_frd = vx_body
        vy_frd = -vy_body
        vz_ned = -vz_body

        vx_ned = cos_y * vx_frd - sin_y * vy_frd
        vy_ned = sin_y * vx_frd + cos_y * vy_frd

        now_us = int(now.nanoseconds / 1000)

        # 1. Offboard Control Mode heartbeat (velocity control)
        ocm = OffboardControlMode()
        ocm.timestamp = now_us
        ocm.position = False
        ocm.velocity = True
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self.pub_ocm.publish(ocm)

        # 2. Trajectory Setpoint in NED
        sp = TrajectorySetpoint()
        sp.timestamp = now_us
        sp.position = [float("nan"), float("nan"), float("nan")]
        sp.velocity = [float(vx_ned), float(vy_ned), float(vz_ned)]
        sp.acceleration = [float("nan"), float("nan"), float("nan")]
        sp.yaw = float("nan")
        # In NED, yaw is CW positive, so CCW rate is negative yawspeed
        sp.yawspeed = float(-r_yaw)
        self.pub_sp.publish(sp)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
