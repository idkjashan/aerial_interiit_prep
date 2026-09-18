#!/usr/bin/env python3
"""ROS 2 node: vision health monitor and EKF2 failsafe watchdog.

Covers the PS requirements "real-time confidence metrics and quality assessment",
"stable health metrics that enable normal arming", and "handle EKF2 failsafe triggers
gracefully, with automatic recovery once valid vision data resumes".

Watches the VO quality signal alongside what EKF2 actually reports back, and publishes a
single consolidated verdict plus an is-it-safe-to-arm flag.

  in : /uav_visual_odometry/quality, /uav_visual_odometry/health
       /fmu/out/vehicle_local_position   (eph/evh + validity, EKF2's own opinion)
       /fmu/out/estimator_status_flags   (is EV actually being fused?)
       /fmu/out/vehicle_status           (arming state, failsafe latch)
       /fmu/out/estimator_status         (optional: innovation test ratios, needs
                                          adding the topic to dds_topics.yaml + rebuild)
  out: ~/status (diagnostic_msgs/DiagnosticArray), ~/ready_to_arm (Bool)

The arming gates being tracked here are the real v1.16 ones:
  * EKF2 pre-flight innovation gate is a hard-coded constant, kMinTestRatioPreflight
    = 0.5 (EKF2.cpp), not a COM_ARM_EKF_* parameter. Those parameters do not exist
    in v1.16; older guides that mention them are out of date.
  * Commander gates on COM_POS_FS_EPH (default 5.0 m) vs vehicle_local_position.eph and
    COM_VEL_FS_EVH (default 1.0 m/s) vs .evh, with 2.5x hysteresis on recovery.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from px4_msgs.msg import VehicleLocalPosition, EstimatorStatusFlags, VehicleStatus

from .qos import px4_sub_qos

try:                                   # only present if dds_topics.yaml was extended
    from px4_msgs.msg import EstimatorStatus
    HAVE_ESTIMATOR_STATUS = True
except ImportError:                    # pragma: no cover
    HAVE_ESTIMATOR_STATUS = False

PREFLIGHT_TEST_RATIO_LIMIT = 0.5       # EKF2.cpp kMinTestRatioPreflight
COM_POS_FS_EPH_DEFAULT = 5.0
COM_VEL_FS_EVH_DEFAULT = 1.0


class VisionHealth(Node):

    def __init__(self):
        super().__init__('uav_vision_health')
        self.declare_parameters('', [
            ('px4_durability', 'volatile'),
            ('eph_limit', COM_POS_FS_EPH_DEFAULT),
            ('evh_limit', COM_VEL_FS_EVH_DEFAULT),
            ('min_quality_for_arm', 40),
            ('stable_seconds_for_arm', 3.0),
            ('subscribe_estimator_status', True),
        ])
        g = lambda n: self.get_parameter(n).value
        self.eph_limit = float(g('eph_limit'))
        self.evh_limit = float(g('evh_limit'))
        self.min_q_arm = int(g('min_quality_for_arm'))
        self.stable_needed = float(g('stable_seconds_for_arm'))

        self.quality = 0
        self.vo_health = 'LOST'
        self.lpos = None
        self.flags = None
        self.vstatus = None
        self.est = None
        self.stable_since = None
        self.failsafe_latched = False
        self.failsafe_events = 0
        self.recovery_events = 0

        q = px4_sub_qos(g('px4_durability'))
        self.create_subscription(Int32, '/uav_visual_odometry/quality',
                                 lambda m: setattr(self, 'quality', int(m.data)), 10)
        self.create_subscription(String, '/uav_visual_odometry/health',
                                 lambda m: setattr(self, 'vo_health', m.data), 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 lambda m: setattr(self, 'lpos', m), q)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                 lambda m: setattr(self, 'lpos', m), q)
        self.create_subscription(EstimatorStatusFlags, '/fmu/out/estimator_status_flags',
                                 lambda m: setattr(self, 'flags', m), q)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 self._on_status, q)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self._on_status, q)
        if HAVE_ESTIMATOR_STATUS and bool(g('subscribe_estimator_status')):
            self.create_subscription(EstimatorStatus, '/fmu/out/estimator_status',
                                     lambda m: setattr(self, 'est', m), q)
        else:
            self.get_logger().warn(
                'estimator_status not subscribed: innovation test ratios will not be '
                'logged. Add it to src/modules/uxrce_dds_client/dds_topics.yaml and '
                'rebuild PX4 to enable the full Phase-2 innovation evidence.')

        self.pub_ready = self.create_publisher(Bool, '~/ready_to_arm', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '~/status', 10)
        self.create_timer(0.2, self._tick)

    def _on_status(self, msg):
        if self.vstatus is not None and msg.failsafe and not self.vstatus.failsafe:
            self.failsafe_events += 1
            self.failsafe_latched = True
            self.get_logger().error(
                f'PX4 FAILSAFE asserted (nav_state={msg.nav_state}); '
                f'vision quality={self.quality} health={self.vo_health}')
        if self.vstatus is not None and self.vstatus.failsafe and not msg.failsafe:
            self.recovery_events += 1
            self.failsafe_latched = False
            self.get_logger().info('PX4 failsafe cleared -- vision fusion recovered')
        self.vstatus = msg

    # ------------------------------------------------------------------ logic
    def _ekf_ok(self):
        """Does EKF2 itself consider the vision-aided local solution usable?"""
        if self.lpos is None:
            return False, 'no vehicle_local_position yet'
        if not (self.lpos.xy_valid and self.lpos.z_valid):
            return False, 'EKF2 local position not valid (xy_valid/z_valid false)'
        if not (self.lpos.v_xy_valid and self.lpos.v_z_valid):
            return False, 'EKF2 local velocity not valid'
        if self.lpos.eph > self.eph_limit:
            return False, f'eph {self.lpos.eph:.2f} m > COM_POS_FS_EPH {self.eph_limit}'
        if self.lpos.evh > self.evh_limit:
            return False, f'evh {self.lpos.evh:.2f} m/s > COM_VEL_FS_EVH {self.evh_limit}'
        if self.lpos.dead_reckoning:
            return False, 'EKF2 is dead-reckoning (no aiding source being fused)'
        if self.flags is not None and not (self.flags.cs_ev_pos or self.flags.cs_ev_vel):
            return False, 'EKF2 is not fusing external vision (cs_ev_pos/cs_ev_vel false)'
        if self.est is not None:
            for name, val in (('pos', self.est.pos_test_ratio),
                              ('vel', self.est.vel_test_ratio),
                              ('hgt', self.est.hgt_test_ratio)):
                if val > PREFLIGHT_TEST_RATIO_LIMIT:
                    return False, f'{name}_test_ratio {val:.2f} > {PREFLIGHT_TEST_RATIO_LIMIT}'
        return True, 'ok'

    def _tick(self):
        ekf_ok, why = self._ekf_ok()
        vision_ok = self.vo_health in ('OK', 'DEGRADED') and self.quality >= self.min_q_arm
        now = self.get_clock().now()
        if ekf_ok and vision_ok:
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None
        stable_for = (0.0 if self.stable_since is None
                      else (now - self.stable_since).nanoseconds * 1e-9)
        ready = stable_for >= self.stable_needed
        self.pub_ready.publish(Bool(data=bool(ready)))

        d = DiagnosticArray()
        d.header.stamp = now.to_msg()
        s = DiagnosticStatus(name='uav_vision/health', hardware_id='ekf2_ev')
        s.level = (DiagnosticStatus.OK if ready else
                   DiagnosticStatus.WARN if vision_ok else DiagnosticStatus.ERROR)
        s.message = ('ready to arm (vision-only)' if ready
                     else (why if not ekf_ok else f'vision {self.vo_health} q={self.quality}'))
        kv = [('vo_health', self.vo_health), ('vo_quality', self.quality),
              ('ekf_ok', ekf_ok), ('reason', why),
              ('stable_for_s', round(stable_for, 1)),
              ('failsafe_events', self.failsafe_events),
              ('recovery_events', self.recovery_events)]
        if self.lpos is not None:
            kv += [('eph', round(float(self.lpos.eph), 3)),
                   ('evh', round(float(self.lpos.evh), 3)),
                   ('dead_reckoning', bool(self.lpos.dead_reckoning)),
                   ('z_agl', round(float(-self.lpos.z), 2))]
        if self.flags is not None:
            kv += [('cs_ev_pos', bool(self.flags.cs_ev_pos)),
                   ('cs_ev_vel', bool(self.flags.cs_ev_vel)),
                   ('cs_ev_hgt', bool(self.flags.cs_ev_hgt)),
                   ('cs_gps_hgt', bool(self.flags.cs_gps_hgt))]
        if self.est is not None:
            kv += [('pos_test_ratio', round(float(self.est.pos_test_ratio), 3)),
                   ('vel_test_ratio', round(float(self.est.vel_test_ratio), 3)),
                   ('hgt_test_ratio', round(float(self.est.hgt_test_ratio), 3))]
        s.values = [KeyValue(key=k, value=str(v)) for k, v in kv]
        d.status.append(s)
        self.pub_diag.publish(d)


def main(args=None):
    rclpy.init(args=args)
    node = VisionHealth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
