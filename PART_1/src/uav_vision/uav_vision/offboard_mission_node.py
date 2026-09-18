#!/usr/bin/env python3
"""ROS 2 node: OFFBOARD mission state machine for the Phase-2 vision-only hover.

Sequence: WAIT_HEALTH -> ARM -> TAKEOFF(10 m) -> HOLD(90 s) -> LAND -> DONE, with a
DEGRADED branch that reacts to loss of vision instead of flying blind.

Two PX4 rules decide the ordering; break either and OFFBOARD is refused without much
of an explanation:
  * OffboardControlMode must already be streaming before the mode switch is requested;
    PX4 requires a recent stream (COM_OF_LOSS_T, default 1.0 s). We send >= 10 setpoints
    before asking for the mode.
  * The commander only allows OFFBOARD with ``position: True`` when local position is
    valid -- which, with no GPS, means EV fusion must already be running and converged.
    So we wait for the health monitor's ready_to_arm and arm normally (no force flag),
    as the PS requires.

Degraded-vision policy (PS: "handle EKF2 failsafe triggers gracefully, with automatic
recovery once valid vision data resumes"): vision counts as lost when VO reports LOST or
its health topic goes quiet for 0.5 s. We then stop commanding position and send a
zero-velocity setpoint, which the estimator can still support. Once vision has been good
for degraded_grace_s we go back to position hold and ask for OFFBOARD again, since PX4
may have left it during the dropout.

During TAKEOFF and HOLD the node also re-requests OFFBOARD (and re-arms) whenever the
vehicle is not in it, once a second. That keeps the demo going after a vision dropout,
but it also means the node takes control back from a PX4 failsafe mode; stop the node
before taking over manually.

In HOLD, velocity commands on /cmd_vel (e.g. teleop_twist_keyboard) take over while they
keep arriving; when they stop, the vehicle holds the new position.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Twist
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleCommandAck, VehicleStatus, VehicleLocalPosition,
                          VehicleAttitude)

from .qos import px4_sub_qos, px4_pub_qos

WAIT_HEALTH, ARMING, TAKEOFF, HOLD, DEGRADED, LANDING, DONE = range(7)
STATE_NAMES = {WAIT_HEALTH: 'WAIT_HEALTH', ARMING: 'ARMING', TAKEOFF: 'TAKEOFF',
               HOLD: 'HOLD', DEGRADED: 'DEGRADED', LANDING: 'LANDING', DONE: 'DONE'}


class OffboardMission(Node):

    def __init__(self):
        super().__init__('uav_offboard_mission')
        self.declare_parameters('', [
            ('px4_durability', 'volatile'),
            ('takeoff_altitude', 10.0),      # m AGL; PS Phase 2 uses 10 m
            ('hold_seconds', 90.0),          # PS Phase 2 gate
            ('setpoint_rate', 20.0),
            ('arrival_radius', 0.5),
            ('land_at_end', True),
            ('require_health', True),
            ('degraded_grace_s', 1.0),
        ])
        g = lambda n: self.get_parameter(n).value
        self.alt = float(g('takeoff_altitude'))
        self.hold_s = float(g('hold_seconds'))
        self.rate = float(g('setpoint_rate'))
        self.arrival_r = float(g('arrival_radius'))
        self.land_at_end = bool(g('land_at_end'))
        self.require_health = bool(g('require_health'))
        self.degraded_grace = float(g('degraded_grace_s'))

        self.state = WAIT_HEALTH
        self.ready = False
        self.vo_health = 'LOST'
        self.quality = 0
        self.status = None
        self.lpos = None
        self.setpoint_count = 0
        self.hold_origin = None
        self.hold_start = None
        self.recovered_since = None
        self.mode_requested_at = None
        self.arm_requested_at = None
        self.land_requested_at = None
        self.max_hold_radius = 0.0
        self.hold_samples = 0
        self.hold_violations = 0
        self.last_xy_reset_counter = None
        self.last_vo_health_rx = None

        # Teleop (/cmd_vel) support
        self.yaw = 0.0
        self.cmd_linear = [0.0, 0.0, 0.0]
        self.cmd_yaw_rate = 0.0
        self.last_cmd_vel_rx = None
        self.teleop_active = False

        q = px4_sub_qos(g('px4_durability'))
        self.create_subscription(Bool, '/uav_vision_health/ready_to_arm',
                                 lambda m: setattr(self, 'ready', bool(m.data)), 10)
        self.create_subscription(String, '/uav_visual_odometry/health',
                                 self._on_health, 10)
        self.create_subscription(Int32, '/uav_visual_odometry/quality',
                                 lambda m: setattr(self, 'quality', int(m.data)), 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 self._on_att, q)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude_v1',
                                 self._on_att, q)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 lambda m: setattr(self, 'status', m), q)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 lambda m: setattr(self, 'status', m), q)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack',
                                 self._on_cmd_ack, q)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self._on_lpos, q)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                 self._on_lpos, q)

        self.pub_ocm = self.create_publisher(OffboardControlMode,
                                             '/fmu/in/offboard_control_mode', px4_pub_qos())
        self.pub_sp = self.create_publisher(TrajectorySetpoint,
                                             '/fmu/in/trajectory_setpoint', px4_pub_qos())
        self.pub_cmd = self.create_publisher(VehicleCommand,
                                             '/fmu/in/vehicle_command', px4_pub_qos())
        self.create_timer(1.0 / self.rate, self._tick)
        self.create_timer(2.0, self._report)

    def _on_cmd_ack(self, ack):
        results = {
            0: 'ACCEPTED',
            1: 'TEMPORARILY_REJECTED',
            2: 'DENIED',
            3: 'UNSUPPORTED',
            4: 'FAILED',
            5: 'IN_PROGRESS',
            6: 'CANCELLED',
        }
        res_str = results.get(ack.result, str(ack.result))
        self.get_logger().info(
            f'VehicleCommandAck: cmd={ack.command} result={res_str} '
            f'param1={ack.result_param1} param2={ack.result_param2}')

    def _on_lpos(self, m):
        if self.hold_origin is not None and self.last_xy_reset_counter is not None:
            if m.xy_reset_counter != self.last_xy_reset_counter:
                dx, dy = float(m.delta_xy[0]), float(m.delta_xy[1])
                self.hold_origin[0] += dx
                self.hold_origin[1] += dy
                self.get_logger().warn(
                    f'EKF2 XY reset: adjusted hold_origin by [{dx:.3f}, {dy:.3f}]')
        self.last_xy_reset_counter = m.xy_reset_counter
        self.lpos = m

    def _on_health(self, msg):
        self.vo_health = msg.data
        self.last_vo_health_rx = self.get_clock().now()

    def _on_att(self, msg):
        q = msg.q  # w, x, y, z in NED/FRD
        # yaw in NED (North=0, CW positive)
        self.yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                              1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2))

    def _on_cmd_vel(self, msg):
        self.cmd_linear = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.cmd_yaw_rate = msg.angular.z
        self.last_cmd_vel_rx = self.get_clock().now()

    # ------------------------------------------------------------- utilities
    def _cmd(self, command, **params):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = command
        for i in range(1, 8):
            setattr(m, f'param{i}', float(params.get(f'param{i}', 0.0)))
        target_sys = int(getattr(self.status, 'system_id', 0)) if self.status else 0
        m.target_system = target_sys
        m.target_component = 1
        m.source_system = target_sys if target_sys > 0 else 1
        m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def _heartbeat(self, position=True, velocity=False):
        m = OffboardControlMode()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.position = position
        m.velocity = velocity
        m.acceleration = m.attitude = m.body_rate = False
        self.pub_ocm.publish(m)

    def _setpoint_position(self, ned_xyz, yaw=0.0):
        m = TrajectorySetpoint()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.position = [float(ned_xyz[0]), float(ned_xyz[1]), float(ned_xyz[2])]
        m.velocity = [float('nan')] * 3
        m.acceleration = [float('nan')] * 3
        m.yaw = float(yaw)
        self.pub_sp.publish(m)

    def _setpoint_hold_velocity(self):
        """Zero-velocity setpoint: survivable when position is untrustworthy."""
        m = TrajectorySetpoint()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.position = [float('nan')] * 3
        m.velocity = [0.0, 0.0, 0.0]
        m.acceleration = [float('nan')] * 3
        m.yaw = float('nan')
        self.pub_sp.publish(m)

    @property
    def armed(self):
        return self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    @property
    def offboard(self):
        return self.status is not None and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    # ------------------------------------------------------------ state machine
    def _tick(self):
        now = self.get_clock().now()
        dt_health = ((now - self.last_vo_health_rx).nanoseconds * 1e-9
                     if self.last_vo_health_rx is not None else 999.0)
        vision_bad = (self.vo_health == 'LOST') or (dt_health > 0.5)

        if self.state == WAIT_HEALTH:
            self._heartbeat()
            self._setpoint_position([0.0, 0.0, 0.0])
            self.setpoint_count += 1
            if (self.ready or not self.require_health) and self.setpoint_count > 20:
                self.get_logger().info('vision healthy and setpoints streaming -> OFFBOARD + ARM')
                self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.mode_requested_at = now
                self.state = ARMING
            return

        if self.state == ARMING:
            self._heartbeat()
            self._setpoint_position([0.0, 0.0, 0.0])
            if not self.offboard:
                if self.mode_requested_at is None or (now - self.mode_requested_at).nanoseconds * 1e-9 >= 1.0:
                    self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                    self.mode_requested_at = now
            elif not self.armed:
                if self.arm_requested_at is None or (now - self.arm_requested_at).nanoseconds * 1e-9 >= 1.0:
                    self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                    self.arm_requested_at = now
            if self.armed and self.offboard:
                self.hold_origin = np.array(
                    [self.lpos.x, self.lpos.y] if self.lpos else [0.0, 0.0])
                self.get_logger().info(f'armed in OFFBOARD -> climbing to {self.alt:.1f} m')
                self.state = TAKEOFF
            elif self.mode_requested_at is not None and \
                    (now - self.mode_requested_at).nanoseconds * 1e-9 > 15.0:
                self.get_logger().error(
                    'OFFBOARD/arm not accepted after 15 s. Check: EV fusion running, '
                    'local position valid, COM_RCL_EXCEPT bit 2 set if flying without RC.')
                self.mode_requested_at = now
            return

        if self.state in (TAKEOFF, HOLD, DEGRADED):
            if vision_bad and self.state != DEGRADED:
                self.recovered_since = None
                self.get_logger().warn('vision LOST -> zero-velocity hold, waiting for recovery')
                self.state = DEGRADED
            elif self.state == DEGRADED:
                # vision has to stay good for degraded_grace_s before we trust position again
                if vision_bad:
                    self.recovered_since = None
                elif self.recovered_since is None:
                    self.recovered_since = now
                elif (now - self.recovered_since).nanoseconds * 1e-9 > self.degraded_grace:
                    self.get_logger().info('vision recovered -> resuming position hold and re-requesting OFFBOARD')
                    self.state = HOLD if self.hold_start else TAKEOFF
                    self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                    self.mode_requested_at = now

        if self.state in (TAKEOFF, HOLD):
            if not self.armed:
                if self.arm_requested_at is None or (now - self.arm_requested_at).nanoseconds * 1e-9 >= 1.0:
                    self.get_logger().warn('Vehicle disarmed in TAKEOFF/HOLD -> requesting re-arm')
                    self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                    self.arm_requested_at = now
            if not self.offboard:
                if self.mode_requested_at is None or (now - self.mode_requested_at).nanoseconds * 1e-9 >= 1.0:
                    self.get_logger().warn(
                        f'In {STATE_NAMES[self.state]} but not in OFFBOARD (nav_state={getattr(self.status, "nav_state", "none")}) -> requesting OFFBOARD')
                    self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                    self.mode_requested_at = now

        if self.state == DEGRADED:
            self._heartbeat(position=False, velocity=True)
            self._setpoint_hold_velocity()
            return

        if self.state == TAKEOFF:
            self._heartbeat()
            tgt = [float(self.hold_origin[0]), float(self.hold_origin[1]), -self.alt]
            self._setpoint_position(tgt)
            if self.lpos is not None and abs(-self.lpos.z - self.alt) < self.arrival_r:
                self.hold_start = now
                self.hold_origin = np.array([self.lpos.x, self.lpos.y])
                self.get_logger().info(
                    f'reached {self.alt:.1f} m -> holding for {self.hold_s:.0f} s at ({self.hold_origin[0]:.2f}, {self.hold_origin[1]:.2f})')
                self.state = HOLD
            return

        if self.state == HOLD:
            # Check if teleop /cmd_vel is active
            teleop_cmd_recent = (
                self.last_cmd_vel_rx is not None and
                (now - self.last_cmd_vel_rx).nanoseconds * 1e-9 < 0.8
            )
            has_velocity = any(abs(v) > 0.01 for v in self.cmd_linear) or abs(self.cmd_yaw_rate) > 0.01

            if teleop_cmd_recent and has_velocity:
                if not self.teleop_active:
                    self.get_logger().info('Teleop engaged via /cmd_vel')
                    self.teleop_active = True
                # Body FLU to world NED
                vx_body, vy_body, vz_body = self.cmd_linear
                cos_y = math.cos(self.yaw)
                sin_y = math.sin(self.yaw)
                vx_frd = vx_body
                vy_frd = -vy_body
                vz_ned = -vz_body
                vx_ned = cos_y * vx_frd - sin_y * vy_frd
                vy_ned = sin_y * vx_frd + cos_y * vy_frd

                self._heartbeat(position=False, velocity=True)
                m = TrajectorySetpoint()
                m.timestamp = int(now.nanoseconds / 1000)
                m.position = [float('nan')] * 3
                m.velocity = [float(vx_ned), float(vy_ned), float(vz_ned)]
                m.acceleration = [float('nan')] * 3
                m.yaw = float('nan')
                m.yawspeed = float(-self.cmd_yaw_rate)
                self.pub_sp.publish(m)
                return

            if self.teleop_active:
                self.teleop_active = False
                if self.lpos is not None:
                    self.hold_origin = np.array([self.lpos.x, self.lpos.y])
                    self.alt = float(-self.lpos.z)
                self.get_logger().info(
                    f'Teleop released -> locking hover at ({self.hold_origin[0]:.2f}, '
                    f'{self.hold_origin[1]:.2f}, {-self.alt:.2f} m)')

            self._heartbeat()
            tgt = [float(self.hold_origin[0]), float(self.hold_origin[1]), -self.alt]
            self._setpoint_position(tgt)
            if self.lpos is not None:
                r = float(np.hypot(self.lpos.x - self.hold_origin[0],
                                   self.lpos.y - self.hold_origin[1]))
                self.max_hold_radius = max(self.max_hold_radius, r)
                self.hold_samples += 1
                if r > 1.5:
                    self.hold_violations += 1
            if (now - self.hold_start).nanoseconds * 1e-9 >= self.hold_s:
                pct = (100.0 * (1 - self.hold_violations / max(self.hold_samples, 1)))
                self.get_logger().info(
                    f'HOLD complete: {self.hold_s:.0f} s, max radius '
                    f'{self.max_hold_radius:.3f} m, inside 1.5 m for {pct:.1f}% of samples '
                    f'=> {"PASS" if self.max_hold_radius < 1.5 else "FAIL"}')
                self.state = LANDING if self.land_at_end else DONE
            return

        if self.state == LANDING:
            if self.land_requested_at is None or (now - self.land_requested_at).nanoseconds * 1e-9 >= 1.0:
                self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.land_requested_at = now
            if not self.armed:
                self.get_logger().info('landed and disarmed')
                self.state = DONE
            return

    def _report(self):
        extra = ''
        if self.state == HOLD and self.hold_start is not None:
            el = (self.get_clock().now() - self.hold_start).nanoseconds * 1e-9
            extra = f' hold={el:.0f}/{self.hold_s:.0f}s max_r={self.max_hold_radius:.2f}m'
        alt = f'{-self.lpos.z:.2f}' if self.lpos else 'n/a'
        self.get_logger().info(
            f'[{STATE_NAMES[self.state]}] armed={self.armed} offboard={self.offboard} '
            f'alt={alt} vision={self.vo_health}/{self.quality}{extra}')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
