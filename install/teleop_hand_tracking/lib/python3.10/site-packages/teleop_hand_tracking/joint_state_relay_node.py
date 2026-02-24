#!/usr/bin/env python3
"""
Relay /joint_states to /dual_arm/joint_states with timestamp validation.
The real robot subscribers listen here.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateRelay(Node):
    def __init__(self):
        super().__init__('joint_state_relay')
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.cb, 10)
        self.pub = self.create_publisher(
            JointState, '/dual_arm/joint_states', 10)
        self.get_logger().info("Joint state relay active.")

    def cb(self, msg: JointState):
        # Stamp with current time if not set (fake controllers sometimes skip this)
        if msg.header.stamp.sec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
