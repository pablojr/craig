#!/usr/bin/env python3
"""Idle animator node.
Publishes a low‑priority gesture (default "idle_sway") to /robot/action
periodically when Craig is not busy. The motion manager will execute the
gesture if no other action is active; otherwise the request is ignored.
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IdleAnimator(Node):
    def __init__(self):
        super().__init__("craig_idle_animator")
        # Parameters
        self.declare_parameter("idle_interval_seconds", 30.0)
        self.declare_parameter("gesture_name", "idle_sway")

        self.interval = self.get_parameter("idle_interval_seconds").get_parameter_value().double_value
        self.gesture = self.get_parameter("gesture_name").get_parameter_value().string_value

        self.publisher = self.create_publisher(String, "/robot/action", 10)
        self.timer = self.create_timer(self.interval, self.publish_idle)
        self.get_logger().info(f"Idle animator started: every {self.interval}s publish '{{\"action\": \"{self.gesture}\"}}'")

    def publish_idle(self):
        msg = String()
        msg.data = json.dumps({"action": self.gesture})
        self.publisher.publish(msg)
        self.get_logger().debug(f"Published idle action: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = IdleAnimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
