#!/usr/bin/env python3
"""Prints the vision health and ready-to-arm state every few seconds."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class SimManagerNode(Node):
    def __init__(self):
        super().__init__('uav_sim_manager')
        self.declare_parameters('', [
            ('check_interval', 2.0),
        ])
        interval = float(self.get_parameter('check_interval').value)
        self.status_sub = self.create_subscription(
            Bool, '/uav_vision_health/ready_to_arm', self._on_arm_ready, 10
        )
        self.health_sub = self.create_subscription(
            String, '/uav_visual_odometry/health', self._on_health, 10
        )
        self.arm_ready = False
        self.health = 'UNKNOWN'
        self.timer = self.create_timer(interval, self._check_health)
        self.get_logger().info('Simulation Manager initialized.')

    def _on_arm_ready(self, msg: Bool):
        self.arm_ready = msg.data

    def _on_health(self, msg: String):
        self.health = msg.data

    def _check_health(self):
        self.get_logger().info(
            f"[SimManager] VO Health: {self.health} | Ready to Arm: {self.arm_ready}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
