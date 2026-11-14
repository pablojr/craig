#!/usr/bin/env python3
"""Translate /jaw position targets into joint states for RViz visualization."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class JawToJointStates(Node):
    def __init__(self):
        super().__init__('jaw_to_joint_states')
        self.declare_parameter('jaw_min', 0.0)
        self.declare_parameter('jaw_max', 0.349)
        self.jaw_min = float(self.get_parameter('jaw_min').value)
        self.jaw_max = float(self.get_parameter('jaw_max').value)
        if self.jaw_min > self.jaw_max:
            raise ValueError('jaw_min must be less than or equal to jaw_max')

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.jaw_sub = self.create_subscription(Float64, '/jaw', self.on_jaw, 10)
        self.get_logger().info(
            f'Converting /jaw (radians) to jaw_joint state in '
            f'[{self.jaw_min}, {self.jaw_max}]'
        )

    def on_jaw(self, msg):
        target = float(msg.data)
        if not math.isfinite(target):
            self.get_logger().warning('Ignoring non-finite /jaw command')
            return

        position = min(self.jaw_max, max(self.jaw_min, target))
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = ['jaw_joint']
        state.position = [position]
        self.joint_state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = JawToJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
