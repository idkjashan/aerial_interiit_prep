#!/usr/bin/env python3
"""ROS 2 node: downward RGB-D visual odometry.

Subscribes camera streams plus the flight controller's attitude and gyro, runs
``vo_core.DownwardVO``, and publishes an ENU odometry estimate with a quality signal.
Deliberately knows nothing about PX4 message formats -- ``px4_odometry_bridge`` owns
that translation, so this node stays independently testable.

  in : /uav/rgb, /uav/depth, /uav/camera_info, /uav/depth/camera_info
       /fmu/out/vehicle_attitude   (attitude, NED<-FRD quaternion)
       /fmu/out/sensor_combined    (gyro_rad, body FRD)
  out: ~/odom          nav_msgs/Odometry           (ENU world, FLU body)
       ~/quality       std_msgs/Int32              0-100
       ~/health        std_msgs/String             OK | DEGRADED | LOST
       ~/diagnostics   diagnostic_msgs/DiagnosticArray
"""
import numpy as np
import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, String, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from px4_msgs.msg import VehicleAttitude, SensorCombined

from .vo_core import DownwardVO, VOParams, health_name, HEALTH_OK, HEALTH_LOST
from .frames import (quat_to_R, R_ned_frd_to_enu_flu, euler_from_R_enu_flu,
                     frd_to_flu, R_to_quat, euler_to_R_enu_flu)
from .depth_utils import (agl_from_depth, K_from_camera_info, K_from_fov,
                          depth_at_rgb_pixels, sanitize)
from .qos import px4_sub_qos, sensor_qos


def image_to_np(msg):
    """Minimal Image -> ndarray. Avoids a hard cv_bridge dependency."""
    enc = msg.encoding
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('32FC1',):
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    if enc in ('16UC1',):
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
    if enc in ('mono8',):
        return buf.reshape(msg.height, msg.width)
    if enc in ('rgb8', 'bgr8'):
        return buf.reshape(msg.height, msg.width, 3)
    if enc in ('rgba8', 'bgra8'):
        return buf.reshape(msg.height, msg.width, 4)[:, :, :3]
    raise ValueError(f'unsupported image encoding: {enc}')


class VoNode(Node):

    def __init__(self):
        super().__init__('uav_visual_odometry')
        p = self.declare_parameters('', [
            ('rgb_topic', '/uav/rgb'),
            ('depth_topic', '/uav/depth'),
            ('rgb_info_topic', '/uav/camera_info'),
            ('depth_info_topic', '/uav/depth/camera_info'),
            ('px4_durability', 'volatile'),
            ('sync_slop', 0.035),
            ('camera_link_rpy', [0.0, 1.5707963, 0.0]),
            ('rgb_hfov_fallback', 1.204),
            ('depth_hfov_fallback', 1.274),
            ('publish_rate_limit', 40.0),
            ('min_features', 60),
            ('degraded_features', 120),
            ('max_corners', 500),
            ('rekey_shift', 0.35),
            ('clahe_clip', 2.0),
            ('vel_lpf_alpha', 0.6),
        ])
        g = lambda n: self.get_parameter(n).value

        params = VOParams()
        params.min_features = int(g('min_features'))
        params.degraded_features = int(g('degraded_features'))
        params.max_corners = int(g('max_corners'))
        params.rekey_shift = float(g('rekey_shift'))
        params.clahe_clip = float(g('clahe_clip'))
        params.vel_lpf_alpha = float(g('vel_lpf_alpha'))
        self._params = params
        self._cam_rpy = tuple(float(v) for v in g('camera_link_rpy'))

        self.vo = None                     # built once intrinsics are known
        self.K_rgb = None
        self.K_depth = None
        self.rpy = (0.0, 0.0, 0.0)
        self.omega_flu = np.zeros(3)
        self.have_attitude = False
        self.last_stamp = None
        self.frames = 0
        self.lat_ms = 0.0

        qos_px4 = px4_sub_qos(g('px4_durability'))
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 self._on_attitude, qos_px4)
        self.create_subscription(SensorCombined, '/fmu/out/sensor_combined',
                                 self._on_imu, qos_px4)
        self.create_subscription(CameraInfo, g('rgb_info_topic'),
                                 lambda m: self._on_info(m, 'rgb'), sensor_qos())
        self.create_subscription(CameraInfo, g('depth_info_topic'),
                                 lambda m: self._on_info(m, 'depth'), sensor_qos())

        rgb_sub = message_filters.Subscriber(self, Image, g('rgb_topic'), qos_profile=sensor_qos())
        dep_sub = message_filters.Subscriber(self, Image, g('depth_topic'), qos_profile=sensor_qos())
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, dep_sub], queue_size=10, slop=float(g('sync_slop')))
        self.sync.registerCallback(self._on_frame)

        self.pub_odom = self.create_publisher(Odometry, '~/odom', 10)
        self.pub_quality = self.create_publisher(Int32, '~/quality', 10)
        self.pub_health = self.create_publisher(String, '~/health', 10)
        self.pub_alt = self.create_publisher(Float32, '~/agl', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self._report)
        self.get_logger().info('visual odometry node up; waiting for camera_info + attitude')

    # ------------------------------------------------------------ callbacks
    def _on_info(self, msg, which):
        K = K_from_camera_info(msg)
        if which == 'rgb' and self.K_rgb is None:
            self.K_rgb = K
            self.get_logger().info(f'RGB intrinsics: fx={K[0,0]:.2f} cx={K[0,2]:.2f} '
                                   f'({msg.width}x{msg.height})')
        elif which == 'depth' and self.K_depth is None:
            self.K_depth = K
            self.get_logger().info(f'depth intrinsics: fx={K[0,0]:.2f} cx={K[0,2]:.2f} '
                                   f'({msg.width}x{msg.height})')

    def _on_attitude(self, msg):
        # PX4 gives NED<-FRD; VO works in ENU/FLU.
        R_enu_flu = R_ned_frd_to_enu_flu(quat_to_R(msg.q))
        self.rpy = euler_from_R_enu_flu(R_enu_flu)
        self.have_attitude = True

    def _on_imu(self, msg):
        self.omega_flu = frd_to_flu(np.asarray(msg.gyro_rad, dtype=float))

    def _on_frame(self, rgb_msg, depth_msg):
        if not self.have_attitude:
            return
        t = rclpy.time.Time.from_msg(rgb_msg.header.stamp).nanoseconds * 1e-9
        if self.last_stamp is None:
            self.last_stamp = t
            return
        dt = t - self.last_stamp
        if dt <= 1e-4 or dt > 1.0:                 # skip duplicate / stale pairs
            self.last_stamp = t
            return
        self.last_stamp = t

        try:
            rgb = image_to_np(rgb_msg)
            depth = image_to_np(depth_msg).astype(np.float32)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        if depth_msg.encoding == '16UC1':
            depth = depth * 1e-3                    # mm -> m
        depth = sanitize(depth)                     # gz emits +/-Inf, not NaN

        if self.K_rgb is None:
            self.K_rgb = K_from_fov(rgb_msg.width, rgb_msg.height,
                                    float(self.get_parameter('rgb_hfov_fallback').value))
            self.get_logger().warn('no RGB camera_info; using SDF-FOV fallback intrinsics')
        if self.K_depth is None:
            self.K_depth = K_from_fov(depth_msg.width, depth_msg.height,
                                      float(self.get_parameter('depth_hfov_fallback').value))
            self.get_logger().warn('no depth camera_info; using SDF-FOV fallback intrinsics')
        if self.vo is None:
            self.vo = DownwardVO(self.K_rgb, self._params, self._cam_rpy)
            self.get_logger().info('VO initialised')

        agl, valid_frac = agl_from_depth(depth, self.rpy[0], self.rpy[1])

        # Features live in RGB pixels; depth lives in its own, differently-scaled image.
        self.vo._depth_at = lambda _d, pts: depth_at_rgb_pixels(
            depth, pts, self.K_rgb, self.K_depth)

        t0 = self.get_clock().now().nanoseconds
        res = self.vo.step(rgb, depth, self.rpy, self.omega_flu, dt, agl)
        self.lat_ms = (self.get_clock().now().nanoseconds - t0) * 1e-6
        self.frames += 1
        self._publish(res, rgb_msg.header.stamp, agl, valid_frac)

    # ------------------------------------------------------------- publish
    def _publish(self, res, stamp, agl, valid_frac):
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'odom_enu'
        od.child_frame_id = 'base_link_flu'
        pos, vel = res['pos'], res['vel']
        od.pose.pose.position.x = float(pos[0])
        od.pose.pose.position.y = float(pos[1])
        od.pose.pose.position.z = float(pos[2])
        q = R_to_quat(euler_to_R_enu_flu(*self.rpy))
        od.pose.pose.orientation.w = float(q[0])
        od.pose.pose.orientation.x = float(q[1])
        od.pose.pose.orientation.y = float(q[2])
        od.pose.pose.orientation.z = float(q[3])
        od.twist.twist.linear.x = float(vel[0])
        od.twist.twist.linear.y = float(vel[1])
        od.twist.twist.linear.z = float(vel[2])
        # Covariance scales with tracking quality: this is the confidence signal the
        # bridge turns into VehicleOdometry variances, and it is how EKF2 learns to
        # de-weight the estimate instead of being fed a silently-bad pose.
        q01 = max(res['quality'], 1) / 100.0
        pvar = float(np.clip(0.02 / (q01 ** 2), 0.02, 25.0))
        vvar = float(np.clip(0.05 / (q01 ** 2), 0.05, 25.0))
        od.pose.covariance[0] = od.pose.covariance[7] = pvar
        od.pose.covariance[14] = 0.02
        od.twist.covariance[0] = od.twist.covariance[7] = od.twist.covariance[14] = vvar
        self.pub_odom.publish(od)
        self.pub_quality.publish(Int32(data=int(res['quality'])))
        self.pub_health.publish(String(data=health_name(res['health'])))
        self.pub_alt.publish(Float32(data=float(agl) if np.isfinite(agl) else float('nan')))

        d = DiagnosticArray()
        d.header.stamp = stamp
        s = DiagnosticStatus(name='uav_vision/vo', hardware_id='downward_rgbd')
        s.level = (DiagnosticStatus.OK if res['health'] == HEALTH_OK else
                   DiagnosticStatus.ERROR if res['health'] == HEALTH_LOST else
                   DiagnosticStatus.WARN)
        s.message = f"{health_name(res['health'])} ({res['mode']})"
        s.values = [KeyValue(key=k, value=str(v)) for k, v in (
            ('tracked_features', res['n_tracked']),
            ('keyframe_inliers', res['n_kf_inliers']),
            ('quality', res['quality']),
            ('keyframe_resets', res['kf_resets']),
            ('agl_m', round(float(agl), 3) if np.isfinite(agl) else 'nan'),
            ('depth_valid_frac', round(valid_frac, 3)),
            ('latency_ms', round(self.lat_ms, 2)))]
        d.status.append(s)
        self.pub_diag.publish(d)

    def _report(self):
        if self.vo is None:
            self.get_logger().warn(
                'no synchronised RGB+depth frames yet -- check the ros_gz_bridge topics '
                'and that /fmu/out/vehicle_attitude is arriving (QoS!)',
                throttle_duration_sec=10.0)
            return
        self.get_logger().info(
            f'frames={self.frames} feat={self.vo.n_tracked} '
            f'health={health_name(self.vo.health)} latency={self.lat_ms:.1f} ms')
        self.frames = 0


def main(args=None):
    rclpy.init(args=args)
    node = VoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
