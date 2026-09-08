#!/usr/bin/env python3
"""ROS 2 node: translate ENU visual odometry into PX4 external-vision odometry.

  in : ~/odom (nav_msgs/Odometry, ENU/FLU), ~/quality (Int32), ~/health (String)
  out: /fmu/in/vehicle_visual_odometry (px4_msgs/VehicleOdometry, NED/FRD)

Everything here exists because of a specific PX4 v1.16 behaviour, verified in source:

* ``pose_frame`` / ``velocity_frame`` MUST be set explicitly. They default-construct to
  0 (UNKNOWN), which fails PX4's frame switch and the field is silently dropped.
* ``timestamp``/``timestamp_sample`` of 0 make PX4 substitute ``hrt_absolute_time()`` on
  arrival (see Tools/msg/templates/ucdr/msg.h.em). That is the most robust choice for
  SITL: it removes the ROS<->PX4 clock-domain problem entirely. The residual pipeline
  latency is then declared once via the EKF2_EV_DELAY parameter instead. Set
  ``stamp_mode:=ros_time`` to send real stamps if you would rather compensate per-message.
* Variances are only a LOWER BOUND while EKF2_EV_NOISE_MD = 0: PX4 takes
  ``max(EKF2_EVx_NOISE^2, msg_variance)``. Sending tiny variances cannot make PX4 trust
  the estimate more than the parameter allows, but LARGE variances do de-weight it --
  which is exactly how degraded vision is communicated.
* A quaternion is rejected unless it is finite, non-zero and normalised to within 1e-5.
  To fuse position/velocity but NOT yaw, send genuine NaN in q -- never zeros.
* ``reset_counter`` must increment ONLY on a true discontinuity (re-init / relocalisation).
  Incrementing every message makes every sample look like a jump; never incrementing
  across a real jump makes EKF2 fight it as an outlier.
* EV must arrive at >= 5 Hz (EKF2 EV_MAX_INTERVAL = 200 ms) or fusion will not start;
  active fusion stops after 400 ms of silence. We run at 30 Hz.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, String
from px4_msgs.msg import VehicleOdometry

from .frames import enu_to_ned, quat_to_R, R_enu_flu_to_ned_frd, R_to_quat
from .qos import px4_pub_qos

NAN3 = [float('nan')] * 3
NAN4 = [float('nan')] * 4


class OdometryBridge(Node):

    def __init__(self):
        super().__init__('uav_px4_odometry_bridge')
        self.declare_parameters('', [
            ('odom_topic', '/uav_visual_odometry/odom'),
            ('quality_topic', '/uav_visual_odometry/quality'),
            ('health_topic', '/uav_visual_odometry/health'),
            ('publish_rate', 30.0),
            ('stamp_mode', 'zero'),          # 'zero' (recommended) | 'ros_time'
            ('fuse_yaw', False),             # keep False while the magnetometer is on
            ('min_quality_to_publish', 1),
            ('max_age_s', 0.25),
            ('pos_var_floor', 0.02),
            ('vel_var_floor', 0.05),
            ('yaw_var', 0.05),
        ])
        g = lambda n: self.get_parameter(n).value
        self.rate = float(g('publish_rate'))
        self.stamp_mode = str(g('stamp_mode'))
        self.fuse_yaw = bool(g('fuse_yaw'))
        self.min_q = int(g('min_quality_to_publish'))
        self.max_age = float(g('max_age_s'))
        self.pvar_floor = float(g('pos_var_floor'))
        self.vvar_floor = float(g('vel_var_floor'))
        self.yaw_var = float(g('yaw_var'))

        self.last_odom = None
        self.last_rx = None
        self.quality = 0
        self.health = 'LOST'
        self.reset_counter = 0
        self.was_healthy = False
        self.sent = 0
        self.skipped = 0

        self.create_subscription(Odometry, g('odom_topic'), self._on_odom, 10)
        self.create_subscription(Int32, g('quality_topic'),
                                 lambda m: setattr(self, 'quality', int(m.data)), 10)
        self.create_subscription(String, g('health_topic'), self._on_health, 10)
        self.pub = self.create_publisher(VehicleOdometry,
                                         '/fmu/in/vehicle_visual_odometry', px4_pub_qos())
        self.create_timer(1.0 / self.rate, self._tick)
        self.create_timer(2.0, self._report)
        self.get_logger().info(
            f'PX4 EV bridge up: {self.rate:.0f} Hz, stamp_mode={self.stamp_mode}, '
            f'fuse_yaw={self.fuse_yaw}')

    def _on_odom(self, msg):
        self.last_odom = msg
        self.last_rx = self.get_clock().now()

    def _on_health(self, msg):
        new = msg.data
        # A LOST -> OK transition means the estimator re-anchored: that is a genuine
        # discontinuity, and the one case where reset_counter must increment.
        if self.health in ('LOST',) and new == 'OK' and self.was_healthy:
            self.reset_counter = (self.reset_counter + 1) % 256
            self.get_logger().warn(
                f'vision recovered after loss -> reset_counter={self.reset_counter}')
        if new == 'OK':
            self.was_healthy = True
        self.health = new

    def _tick(self):
        if self.last_odom is None or self.last_rx is None:
            self.skipped += 1
            return
        age = (self.get_clock().now() - self.last_rx).nanoseconds * 1e-9
        if age > self.max_age or self.quality < self.min_q:
            self.skipped += 1          # stale/unusable: publish nothing, let EKF2 time out
            return

        o = self.last_odom
        m = VehicleOdometry()
        if self.stamp_mode == 'ros_time':
            now_us = self.get_clock().now().nanoseconds // 1000
            m.timestamp = int(now_us)
            m.timestamp_sample = int(
                rclpy.time.Time.from_msg(o.header.stamp).nanoseconds // 1000)
        else:
            m.timestamp = 0            # PX4 substitutes hrt_absolute_time() on arrival
            m.timestamp_sample = 0

        p = o.pose.pose.position
        v = o.twist.twist.linear
        pos_ned = enu_to_ned([p.x, p.y, p.z])
        vel_ned = enu_to_ned([v.x, v.y, v.z])

        m.pose_frame = VehicleOdometry.POSE_FRAME_NED
        m.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        m.position = [float(c) for c in pos_ned]
        m.velocity = [float(c) for c in vel_ned]
        m.angular_velocity = NAN3                    # not estimated by vision

        if self.fuse_yaw:
            q = o.pose.pose.orientation
            R_ned_frd = R_enu_flu_to_ned_frd(quat_to_R([q.w, q.x, q.y, q.z]))
            qn = R_to_quat(R_ned_frd)
            qn = qn / np.linalg.norm(qn)             # PX4 rejects |1-norm| > 1e-5
            m.q = [float(c) for c in qn]
            m.orientation_variance = [self.yaw_var] * 3
        else:
            m.q = NAN4                               # NaN, not zeros: zeros are rejected
            m.orientation_variance = NAN3

        # Confidence -> variance. Lower quality => larger variance => EKF2 de-weights.
        c = o.pose.covariance
        pv = max(float(c[0]), self.pvar_floor) if math.isfinite(c[0]) else self.pvar_floor
        vv = (max(float(o.twist.covariance[0]), self.vvar_floor)
              if math.isfinite(o.twist.covariance[0]) else self.vvar_floor)
        m.position_variance = [pv, pv, max(float(c[14]), self.pvar_floor)]
        m.velocity_variance = [vv, vv, vv]
        m.reset_counter = int(self.reset_counter)
        m.quality = int(np.clip(self.quality, 0, 100))

        if not (np.all(np.isfinite(m.position)) and np.all(np.isfinite(m.velocity))):
            self.skipped += 1                        # never feed EKF2 a NaN position
            return
        self.pub.publish(m)
        self.sent += 1

    def _report(self):
        self.get_logger().info(
            f'EV published={self.sent} skipped={self.skipped} '
            f'quality={self.quality} health={self.health} resets={self.reset_counter}')
        if self.sent == 0:
            self.get_logger().warn(
                'no EV messages sent -- EKF2 needs >=5 Hz to start fusion. '
                'Check that the VO node is publishing odom and quality > 0.')
        self.sent = self.skipped = 0


def main(args=None):
    rclpy.init(args=args)
    node = OdometryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
