#!/usr/bin/env python3
"""ROS 2 node: OFFBOARD mission state machine for the Phase-2 vision-only hover.

Sequence: WAIT_HEALTH -> ARM -> TAKEOFF(10 m) -> HOLD(90 s) -> LAND -> DONE, with a
DEGRADED branch that reacts to loss of vision instead of flying blind.

Two PX4 rules drive the ordering, and getting either wrong is the classic reason
OFFBOARD is silently refused:
  * OffboardControlMode must already be streaming BEFORE the mode switch is requested;
    PX4 requires a recent stream (COM_OF_LOSS_T, default 1.0 s). We send >= 10 setpoints
    before asking for the mode.
  * The commander only allows OFFBOARD with ``position: True`` when local position is
    valid -- which, with no GPS, means EV fusion must already be running and converged.
    That is why we wait on the health monitor's ready_to_arm before arming rather than
    force-arming, which is precisely what the PS asks for.

Degraded-vision policy (PS: "handle EKF2 failsafe triggers gracefully, with automatic
recovery once valid vision data resumes"): on loss of vision we stop commanding position
and switch to a zero-velocity setpoint, which keeps OFFBOARD alive on a signal the
estimator can still support, then return to position hold once vision recovers. We do
NOT disarm and we do not fight a PX4-initiated failsafe.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleCommandAck, VehicleStatus, VehicleLocalPosition)

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
        self.degraded_since = None
        self.mode_requested_at = None
        self.arm_requested_at = None
        self.land_requested_at = None
        self.max_hold_radius = 0.0
        self.hold_samples = 0
        self.hold_violations = 0
        self.last_xy_reset_counter = None

        q = px4_sub_qos(g('px4_durability'))
        self.create_subscription(Bool, '/uav_vision_health/ready_to_arm',
                                 lambda m: setattr(self, 'ready', bool(m.data)), 10)
        self.create_subscription(String, '/uav_visual_odometry/health',
                                 lambda m: setattr(self, 'vo_health', m.data), 10)
        self.create_subscription(Int32, '/uav_visual_odometry/quality',
                                 lambda m: setattr(self, 'quality', int(m.data)), 10)
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
        vision_bad = self.vo_health == 'LOST'

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
                self.degraded_since = now
                self.get_logger().warn('vision LOST -> zero-velocity hold, waiting for recovery')
                self.state = DEGRADED
            elif self.state == DEGRADED and not vision_bad:
                if (now - self.degraded_since).nanoseconds * 1e-9 > self.degraded_grace:
                    self.get_logger().info('vision recovered -> resuming position hold')
                    self.state = HOLD if self.hold_start else TAKEOFF

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
