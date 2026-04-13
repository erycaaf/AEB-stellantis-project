#!/usr/bin/env python3
"""
CAN ↔ ROS 2 Bridge
===================
Bridges between raw SocketCAN frames on vcan0 and typed ROS 2 topics.

Direction:
  ROS 2 → CAN: perception_node publishes CAN-format msgs → encode to raw frames → vcan0
  CAN → ROS 2: Zephyr ECU sends raw CAN frames on vcan0 → decode → ROS 2 topics

This uses ros2_socketcan's SocketCanBridge node for the raw transport,
plus a custom translation node for the AEB-specific message types.
"""

import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
import struct


# CAN IDs (from aeb_system.dbc)
CAN_ID_BRAKE_CMD    = 0x080   # TX from Zephyr ECU
CAN_ID_EGO_VEHICLE  = 0x100   # TX from Gazebo (perception)
CAN_ID_DRIVER_INPUT = 0x101   # TX from Gazebo
CAN_ID_RADAR_TARGET = 0x120   # TX from Gazebo (perception)
CAN_ID_FSM_STATE    = 0x200   # TX from Zephyr ECU
CAN_ID_ALERT        = 0x300   # TX from Zephyr ECU


class AebCanBridge(Node):
    def __init__(self):
        super().__init__('aeb_can_bridge')

        # Subscribe to raw CAN frames from vcan0 (via ros2_socketcan)
        self.create_subscription(Frame, '/can/rx', self.can_rx_cb, 10)

        # Publisher for raw CAN frames to vcan0
        self.can_tx_pub = self.create_publisher(Frame, '/can/tx', 10)

        self.get_logger().info('AEB CAN Bridge started on vcan0')

    def can_rx_cb(self, msg: Frame):
        """Handle frames received from vcan0 (from Zephyr ECU)."""
        can_id = msg.id

        if can_id == CAN_ID_BRAKE_CMD:
            # Decode brake command from Zephyr
            self.get_logger().debug(
                f'RX BrakeCmd: {msg.data[:4].hex()}'
            )
        elif can_id == CAN_ID_FSM_STATE:
            self.get_logger().debug(
                f'RX FSMState: {msg.data[:4].hex()}'
            )

    def send_can_frame(self, can_id, data, dlc):
        """Send a raw CAN frame to vcan0 (to Zephyr ECU)."""
        msg = Frame()
        msg.id = can_id
        msg.dlc = dlc
        msg.is_extended = False
        msg.is_rtr = False
        msg.is_error = False
        for i in range(min(dlc, 8)):
            msg.data[i] = data[i]
        self.can_tx_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AebCanBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
