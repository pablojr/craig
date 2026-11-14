#!/usr/bin/env python3
"""Translate /jaw position targets into a complete joint-state message for RViz."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from urdf_parser_py import urdf as urdf_parser


class JawStateMerger(Node):
    def __init__(self):
        super().__init__('jaw_state_merger')
        self.declare_parameter('jaw_min', 0.0)
        self.declare_parameter('jaw_max', 0.349)
        self.jaw_min = float(self.get_parameter('jaw_min').value)
        self.jaw_max = float(self.get_parameter('jaw_max').value)

        self.robot = urdf_parser.URDF.from_parameter_server('/robot_description')
        self.joint_names = [joint.name for joint in self.robot.joints if joint.type != 'fixed']
        self.current_positions = {name: 0.0 for name in self.joint_names}

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.jaw_sub = self.create_subscription(Float64, '/jaw', self.on_jaw, 10)
        self.get_logger().info(
            f'Publishing full joint state while driving jaw_joint from /jaw '
            f'in [{self.jaw_min}, {self.jaw_max}]'
        )

    def on_jaw(self, msg):
        target = float(msg.data)
        if not math.isfinite(target):
            self.get_logger().warning('Ignoring non-finite /jaw command')
            return

        target = max(self.jaw_min, min(self.jaw_max, target))
        self.current_positions['jaw_joint'] = target

        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = list(self.joint_names)
        state.position = [self.current_positions[name] for name in self.joint_names]
        state.velocity = []
        state.effort = []
        self.joint_state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = JawStateMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
