#!/usr/bin/env python3
"""
Target Vehicle Controller
=========================
Placeholder node for direct target vehicle control.
In most scenarios, the scenario_controller handles target motion.
This node can be used for manual/interactive target control.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TargetVehicleController(Node):
    def __init__(self):
        super().__init__('target_vehicle_controller')
        self.declare_parameter('speed_kmh', 0.0)
        self.speed_ms = self.get_parameter('speed_kmh').value / 3.6

        self.cmd_pub = self.create_publisher(Twist, '/aeb/target/cmd_vel', 10)
        self.timer = self.create_timer(0.01, self.publish_cmd)

        self.get_logger().info(f'Target vehicle: {self.speed_ms*3.6:.0f} km/h')

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.speed_ms
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetVehicleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
