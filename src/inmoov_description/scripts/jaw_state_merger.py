#!/usr/bin/env python3
"""Publish a complete RViz joint state with jaw_joint driven by /jaw."""

import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


class JawStateMerger(Node):
    def __init__(self):
        super().__init__('jaw_state_merger')
        self.declare_parameter('jaw_min', 0.0)
        self.declare_parameter('jaw_max', 0.349)
        self.jaw_min = float(self.get_parameter('jaw_min').value)
        self.jaw_max = float(self.get_parameter('jaw_max').value)
        if self.jaw_min > self.jaw_max:
            raise ValueError('jaw_min must be less than or equal to jaw_max')

        self.joint_names = []
        self.positions = {}
        self.jaw_position = min(self.jaw_max, max(self.jaw_min, 0.0))

        description_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.description_sub = self.create_subscription(
            String, '/robot_description', self.on_robot_description, description_qos
        )
        self.jaw_sub = self.create_subscription(Float64, '/jaw', self.on_jaw, 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / 30.0, self.publish_state)
        self.get_logger().info('Waiting for /robot_description; /jaw targets are in radians')

    def on_robot_description(self, msg):
        try:
            root = ET.fromstring(msg.data)
            movable = []
            defaults = {}
            for joint in root.findall('joint'):
                if joint.get('type') == 'fixed':
                    continue
                name = joint.get('name')
                if not name:
                    continue
                movable.append(name)
                value = 0.0
                limit = joint.find('limit')
                if limit is not None:
                    try:
                        lower = float(limit.get('lower', '-inf'))
                        upper = float(limit.get('upper', 'inf'))
                        lower, upper = sorted((lower, upper))
                        value = min(upper, max(lower, value))
                    except (TypeError, ValueError):
                        pass
                defaults[name] = value

            if 'jaw_joint' not in defaults:
                raise ValueError('URDF does not define jaw_joint')
            self.joint_names = movable
            self.positions = defaults
            self.get_logger().info(
                f'Loaded {len(self.joint_names)} movable joints from /robot_description'
            )
        except (ET.ParseError, ValueError) as exc:
            self.get_logger().error(f'Could not parse /robot_description: {exc}')

    def on_jaw(self, msg):
        target = float(msg.data)
        if not math.isfinite(target):
            self.get_logger().warning('Ignoring non-finite /jaw command')
            return
        self.jaw_position = min(self.jaw_max, max(self.jaw_min, target))

    def publish_state(self):
        if not self.joint_names:
            return
        self.positions['jaw_joint'] = self.jaw_position
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = list(self.joint_names)
        state.position = [self.positions[name] for name in self.joint_names]
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
