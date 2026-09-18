#!/usr/bin/env python3
"""Manual flight commands and keyboard control for the Part 1 vehicle.

Provides both an interactive keyboard interface and direct CLI commands for:
  - Arming / Disarming
  - Taking off to arbitrary altitude
  - Hovering / Holding position
  - Sending 3D waypoint goals (goto X Y Z)
  - Safe Landing
  - Keyboard teleoperation (/cmd_vel)
  - Telemetry & Health status inspection

Meant for the manual flight mode started by launch_sim.sh (no mission node running).

Usage:
  Interactive Mode:
    python3 PART_1/tools/manual_control.py

  CLI Commands:
    python3 PART_1/tools/manual_control.py status
    python3 PART_1/tools/manual_control.py arm
    python3 PART_1/tools/manual_control.py takeoff [altitude_m]   # default: 10.0
    python3 PART_1/tools/manual_control.py hover
    python3 PART_1/tools/manual_control.py goto <x> <y> <alt>     # in meters (ENU)
    python3 PART_1/tools/manual_control.py land
    python3 PART_1/tools/manual_control.py disarm
"""
import sys
import os

# Run from a plain shell without sourcing anything: restart once with px4_msgs on
# LD_LIBRARY_PATH, and use the launch scripts' ROS_DOMAIN_ID (77) unless one is set.
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

import time
import math
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist, PoseStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
    VehicleAttitude,
)


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
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class ManualController(Node):
    def __init__(self):
        super().__init__("uav_manual_controller")

        self.status = None
        self.lpos = None
        self.yaw = 0.0
        self.have_att = False
        self.vo_health = "UNKNOWN"
        self.quality = 0

        # Subscriptions
        q = px4_sub_qos()
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", lambda m: setattr(self, "status", m), q)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", lambda m: setattr(self, "status", m), q)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", lambda m: setattr(self, "lpos", m), q)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", lambda m: setattr(self, "lpos", m), q)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self._on_att, q)
        self.create_subscription(String, "/uav_visual_odometry/health", self._on_health, 10)
        self.create_subscription(Int32, "/uav_visual_odometry/quality", lambda m: setattr(self, "quality", m.data), 10)

        # Publishers
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", px4_pub_qos())
        self.pub_ocm = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", px4_pub_qos())
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", px4_pub_qos())
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal_pose, 10)

        # Standalone control state
        self.mode = "IDLE"  # IDLE, TAKEOFF, HOVER, GOTO, TELEOP
        self.target_pos = [0.0, 0.0, -10.0]  # NED
        self.target_yaw = 0.0                 # NED heading held with the position setpoint
        self.cmd_linear = [0.0, 0.0, 0.0]
        self.cmd_yaw_rate = 0.0
        self.last_key_time = None

        self.ready_to_arm = False
        from std_msgs.msg import Bool
        self.create_subscription(Bool, "/uav_vision_health/ready_to_arm", lambda m: setattr(self, "ready_to_arm", bool(m.data)), 10)

        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz

    @property
    def is_armed(self):
        return self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    @property
    def is_offboard(self):
        return self.status is not None and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def _on_att(self, msg):
        q = msg.q
        self.have_att = True
        self.yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2))

    def _on_cmd_vel(self, msg):
        self.mode = "TELEOP"
        self.cmd_linear = [float(msg.linear.x), float(msg.linear.y), float(msg.linear.z)]
        self.cmd_yaw_rate = float(msg.angular.z)
        self.last_key_time = self.get_clock().now().nanoseconds * 1e-9

    def _on_goal_pose(self, msg):
        alt = float(msg.pose.position.z) if msg.pose.position.z > 0.5 else (-self.target_pos[2] if self.target_pos[2] < 0 else 10.0)
        self.get_logger().info(f"Received /goal_pose: ENU [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {alt:.2f}]")
        self.goto(msg.pose.position.x, msg.pose.position.y, alt)

    def _on_health(self, msg):
        old = self.vo_health
        self.vo_health = str(msg.data)
        if old == "LOST" and self.vo_health != "LOST":
            self.get_logger().info("Vision RECOVERED -> Re-asserting OFFBOARD mode and resuming position hold")
            if self.lpos and self.mode == "HOVER":
                self.target_pos = [float(self.lpos.x), float(self.lpos.y), float(self.target_pos[2])]
            self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def send_cmd(self, command, **params):
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

    def arm(self):
        sp = self.target_pos if self.mode in ("TAKEOFF", "HOVER", "GOTO") else [0.0, 0.0, 0.0]
        self.get_logger().info("Streaming setpoints and waiting for preflight checks...")
        for _ in range(40):
            self.send_heartbeat(position=True)
            self.send_position_sp(sp)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.ready_to_arm:
                break

        for attempt in range(10):
            self.send_heartbeat(position=True)
            self.send_position_sp(sp)
            self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            time.sleep(0.05)
            self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.is_armed and self.is_offboard:
                    self.get_logger().info("ARMED successfully in OFFBOARD mode!")
                    return True
            time.sleep(0.5)

        self.get_logger().warn("Arming attempt complete. Check status.")
        return False

    def disarm(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("DISARM command sent")

    def land(self):
        self.mode = "IDLE"
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("LAND command sent")

    def takeoff(self, altitude=10.0):
        if self.lpos:
            self.target_pos = [float(self.lpos.x), float(self.lpos.y), -abs(float(altitude))]
        else:
            self.target_pos = [0.0, 0.0, -abs(float(altitude))]
        self.target_yaw = self.yaw
        self.mode = "TAKEOFF"
        self.arm()
        self.get_logger().info(f"Climbing to {altitude:.1f} m AGL...")

    def hover(self):
        if self.lpos:
            self.target_pos = [float(self.lpos.x), float(self.lpos.y), float(self.lpos.z)]
        self.target_yaw = self.yaw
        self.mode = "HOVER"
        self.get_logger().info(f"Holding hover at current position: NED [{self.target_pos[0]:.2f}, {self.target_pos[1]:.2f}, {self.target_pos[2]:.2f}]")

    def goto(self, x_enu, y_enu, alt_m):
        # ENU (x east, y north) -> PX4 local NED (x north, y east, z down)
        x_ned = float(y_enu)
        y_ned = float(x_enu)
        z_ned = -abs(float(alt_m))
        self.target_pos = [x_ned, y_ned, z_ned]
        self.target_yaw = self.yaw
        self.mode = "GOTO"
        self.get_logger().info(f"Navigating to ENU [{x_enu:.2f}, {y_enu:.2f}, {alt_m:.2f}] -> NED [{x_ned:.2f}, {y_ned:.2f}, {z_ned:.2f}]")

    def send_heartbeat(self, position=True, velocity=False):
        m = OffboardControlMode()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.position = position
        m.velocity = velocity
        m.acceleration = m.attitude = m.body_rate = False
        self.pub_ocm.publish(m)

    def send_position_sp(self, ned_xyz, yaw=0.0):
        m = TrajectorySetpoint()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.position = [float(ned_xyz[0]), float(ned_xyz[1]), float(ned_xyz[2])]
        m.velocity = [float("nan")] * 3
        m.acceleration = [float("nan")] * 3
        m.yaw = float(yaw)
        self.pub_sp.publish(m)

    def send_velocity_sp(self, vx_ned, vy_ned, vz_ned, yawspeed=0.0):
        m = TrajectorySetpoint()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.position = [float("nan")] * 3
        m.velocity = [float(vx_ned), float(vy_ned), float(vz_ned)]
        m.acceleration = [float("nan")] * 3
        m.yaw = float("nan")
        m.yawspeed = float(yawspeed)
        self.pub_sp.publish(m)

    def _tick(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9

        # In interactive teleop mode: check key timeout
        if self.mode == "TELEOP":
            if self.last_key_time is None or (now_s - self.last_key_time) > 0.6:
                # Idle brake
                self.cmd_linear = [0.0, 0.0, 0.0]
                self.cmd_yaw_rate = 0.0

            # Publish /cmd_vel for any listening node
            tw = Twist()
            tw.linear.x = float(self.cmd_linear[0])
            tw.linear.y = float(self.cmd_linear[1])
            tw.linear.z = float(self.cmd_linear[2])
            tw.angular.z = float(self.cmd_yaw_rate)
            self.pub_cmd_vel.publish(tw)

            # Also publish direct trajectory setpoint if standalone
            vx_body, vy_body, vz_body = self.cmd_linear
            cos_y = math.cos(self.yaw)
            sin_y = math.sin(self.yaw)
            vx_ned = cos_y * vx_body - sin_y * (-vy_body)
            vy_ned = sin_y * vx_body + cos_y * (-vy_body)
            vz_ned = -vz_body

            self.send_heartbeat(position=False, velocity=True)
            self.send_velocity_sp(vx_ned, vy_ned, vz_ned, yawspeed=-self.cmd_yaw_rate)
            return

        if self.mode in ("TAKEOFF", "HOVER", "GOTO"):
            if self.vo_health == "LOST":
                # Vision lost failsafe: command zero-velocity hold to safely maintain altitude without auto-landing
                self.send_heartbeat(position=False, velocity=True)
                self.send_velocity_sp(0.0, 0.0, 0.0)
                return

            if self.mode == "TAKEOFF" and self.lpos is not None:
                current_alt = -float(self.lpos.z)
                target_alt = abs(float(self.target_pos[2]))
                if current_alt > 1.0 and abs(current_alt - target_alt) < 0.5:
                    self.mode = "HOVER"
                    self.target_pos = [float(self.lpos.x), float(self.lpos.y), -target_alt]
                    self.get_logger().info(f"Target altitude reached. Holding hover at {target_alt:.1f} m.")

            self.send_heartbeat(position=True, velocity=False)
            self.send_position_sp(self.target_pos, self.target_yaw)

    def get_status_str(self):
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED if self.status else False
        nav = self.status.nav_state if self.status else "UNKNOWN"
        x = f"{self.lpos.x:.2f}" if self.lpos else "n/a"
        y = f"{self.lpos.y:.2f}" if self.lpos else "n/a"
        alt = f"{-self.lpos.z:.2f}" if self.lpos else "n/a"
        return (
            f"Armed: {armed} | NavState: {nav} | Alt: {alt} m | "
            f"Pos(X,Y): [{x}, {y}] | VO Health: {self.vo_health} ({self.quality}%)"
        )


def run_interactive(node):
    # Set stdin to non-blocking raw mode
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    help_banner = """
=============================================================
  UAV Manual Flight Controller & Teleoperation
=============================================================
  [Flight Commands]
    a : Arm & Enter Offboard
    t : Takeoff to 10 m (Hold altitude)
    h : Hover / Hold Current Position
    L : Land (shift+l)
    d : Disarm

  [Teleoperation Movement - Body FLU]
    i / , : Forward / Backward (0.8 m/s)
    j / l : Strafe Left / Right (0.8 m/s)
    w / s : Ascend / Descend (0.6 m/s)
    u / o : Yaw Left / Right (0.5 rad/s)
    k     : Brake / Zero Velocity (Hold Position)

  [Other]
    space : Print status
    q     : Quit
=============================================================
"""
    print(help_banner)

    try:
        linear_speed = 0.8
        climb_speed = 0.6
        yaw_rate = 0.5

        last_status_print = 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

            now = time.time()
            if now - last_status_print > 2.0:
                print(f"\rStatus: {node.get_status_str()}   ", end="", flush=True)
                last_status_print = now

            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == "q":
                    break
                elif ch == "a":
                    print("\n[CMD] Arming...")
                    node.arm()
                elif ch == "t":
                    print("\n[CMD] Takeoff to 10m...")
                    node.takeoff(10.0)
                elif ch == "h":
                    print("\n[CMD] Hover...")
                    node.hover()
                elif ch == "L":                      # capital: 'l' is strafe right
                    print("\n[CMD] Landing...")
                    node.land()
                elif ch == "d":
                    print("\n[CMD] Disarming...")
                    node.disarm()
                elif ch == " ":
                    print(f"\nStatus: {node.get_status_str()}")
                elif ch == "i":
                    node.mode = "TELEOP"
                    node.cmd_linear = [linear_speed, 0.0, 0.0]
                    node.cmd_yaw_rate = 0.0
                    node.last_key_time = node.get_clock().now().nanoseconds * 1e-9
                elif ch == ",":
                    node.mode = "TELEOP"
                    node.cmd_linear = [-linear_speed, 0.0, 0.0]
                    node.cmd_yaw_rate = 0.0
                    node.last_key_time = node.get_clock().now().nanoseconds * 1e-9
                elif ch == "j":
                    node.mode = "TELEOP"
                    node.cmd_linear = [0.0, linear_speed, 0.0]
                    node.cmd_yaw_rate = 0.0
                    node.last_key_time = node.get_clock().now().nanoseconds * 1e-9
                elif ch == "l":
                    node.mode = "TELEOP"
                    node.cmd_linear = [0.0, -linear_speed, 0.0]
                    node.cmd_yaw_rate = 0.0
                    node.last_key_time = node.get_clock().now().nanoseconds * 1e-9
                elif ch == "w":
                    node.mode = "TELEOP"
                    node.cmd_linear = [0.0, 0.0, climb_speed]
                    node.cmd_yaw_rate = 0.0
                    node.last_key_time = node.get_clock().now().nanoseconds * 1e-9
                elif ch == "s":
                    node.mode = "TELEOP"
                    node.cmd_linear = [0.0, 0.0, -climb_speed]
                    node.cmd_yaw_rate = 0.0
                    node.last_key_time = node.get_clock().now().nanoseconds * 1e-9
                elif ch == "u":
                    node.mode = "TELEOP"
                    node.cmd_yaw_rate = yaw_rate
                    node.last_key_time = node.get_clock().now().nanoseconds * 1e-9
                elif ch == "o":
                    node.mode = "TELEOP"
                    node.cmd_yaw_rate = -yaw_rate
                    node.last_key_time = node.get_clock().now().nanoseconds * 1e-9
                elif ch == "k":
                    node.cmd_linear = [0.0, 0.0, 0.0]
                    node.cmd_yaw_rate = 0.0
                    node.hover()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    node = ManualController()

    if len(sys.argv) > 1:
        cmd = sys.argv[1].lower().strip("-")
        start_wait = time.time()
        while time.time() - start_wait < 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.status is not None and node.lpos is not None and node.have_att:
                break

        if cmd == "status":
            print(node.get_status_str())
        elif cmd == "arm":
            node.arm()
        elif cmd == "disarm":
            node.disarm()
        elif cmd == "takeoff":
            alt = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
            node.takeoff(alt)
            # Spin while maintaining setpoints
            print(f"Holding takeoff setpoint to {alt}m. Press Ctrl-C to release.")
            try:
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.05)
            except KeyboardInterrupt:
                pass
        elif cmd == "hover":
            node.hover()
            print("Holding current position. Press Ctrl-C to release.")
            try:
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.05)
            except KeyboardInterrupt:
                pass
        elif cmd == "goto":
            if len(sys.argv) < 5:
                print("Usage: manual_control.py goto <x_enu> <y_enu> <alt_m>")
                sys.exit(1)
            x, y, alt = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
            node.goto(x, y, alt)
            print(f"Navigating to [{x}, {y}, {alt}]. Press Ctrl-C to release.")
            try:
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.05)
            except KeyboardInterrupt:
                pass
        elif cmd == "land":
            node.land()
        else:
            print(f"Unknown command: {cmd}")
            print("Available: status, arm, takeoff, hover, goto, land, disarm")
    else:
        run_interactive(node)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
