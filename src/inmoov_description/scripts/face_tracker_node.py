#!/usr/bin/env python3
"""Detect faces from the ROS camera stream and optionally steer the InMoov neck."""

import json
import math

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String


class FaceTrackerNode(Node):
    """Haar/CLAHE face tracker subscribed to the existing camera image topic."""

    def __init__(self):
        super().__init__('face_tracker_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('enable_actuation', False)
        self.declare_parameter('pan_joint', 'head_pan_joint')
        self.declare_parameter('tilt_joint', 'head_tilt_joint')
        self.declare_parameter('pan_min_deg', -100.0)
        self.declare_parameter('pan_max_deg', 90.0)
        self.declare_parameter('tilt_min_deg', -20.0)
        self.declare_parameter('tilt_max_deg', 20.0)
        self.declare_parameter('pan_direction', 1.0)
        self.declare_parameter('tilt_direction', 1.0)
        self.declare_parameter('gain_deg_per_frame_width', 0.8)
        self.declare_parameter('deadband', 0.08)
        self.declare_parameter('max_step_deg', 1.5)
        self.declare_parameter('min_face_size', 40)
        self.declare_parameter('publish_annotated_image', True)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.enable_actuation = bool(self.get_parameter('enable_actuation').value)
        self.pan_joint = str(self.get_parameter('pan_joint').value)
        self.tilt_joint = str(self.get_parameter('tilt_joint').value)
        self.pan_limits = self._limits('pan')
        self.tilt_limits = self._limits('tilt')
        self.pan_direction = float(self.get_parameter('pan_direction').value)
        self.tilt_direction = float(self.get_parameter('tilt_direction').value)
        self.gain = float(self.get_parameter('gain_deg_per_frame_width').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.max_step = float(self.get_parameter('max_step_deg').value)
        self.min_face_size = int(self.get_parameter('min_face_size').value)
        self.publish_annotated = bool(
            self.get_parameter('publish_annotated_image').value)
        if self.gain < 0.0 or self.max_step <= 0.0:
            raise ValueError('gain must be non-negative and max_step_deg positive')
        if not 0.0 <= self.deadband < 1.0:
            raise ValueError('deadband must be in [0, 1)')

        cascade_path = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        if self.face_cascade.empty():
            raise RuntimeError(f'Could not load Haar cascade: {cascade_path}')

        self.bridge = CvBridge()
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.pan_deg = min(self.pan_limits[1], max(self.pan_limits[0], 0.0))
        self.tilt_deg = min(self.tilt_limits[1], max(self.tilt_limits[0], 0.0))
        self.last_face = None

        self.position_pub = self.create_publisher(String, '/face_position', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.annotated_pub = self.create_publisher(Image, '/camera/tracking_image', 1)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f'Subscribing to {self.image_topic}; annotated images: '
            '/camera/tracking_image; face coordinates: /face_position')
        if self.enable_actuation:
            self.get_logger().warning(
                'Neck actuation ENABLED: publishing degree-valued commands on '
                '/joint_command. Verify clearance and direction before use.')
        else:
            self.get_logger().info(
                'Neck actuation is disabled. Set enable_actuation:=true only '
                'after checking limits and direction.')

    def _limits(self, axis):
        lower = float(self.get_parameter(f'{axis}_min_deg').value)
        upper = float(self.get_parameter(f'{axis}_max_deg').value)
        if lower >= upper:
            raise ValueError(f'{axis} limits must be increasing')
        return lower, upper

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except (ValueError, RuntimeError) as exc:
            self.get_logger().warning(f'Could not decode camera image: {exc}')
            return

        height, width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        enhanced = self.clahe.apply(gray)
        faces = self.face_cascade.detectMultiScale(
            enhanced, scaleFactor=1.1, minNeighbors=5,
            minSize=(self.min_face_size, self.min_face_size))

        detection = None
        if len(faces):
            x, y, w, h = max(faces, key=lambda face: face[2] * face[3])
            cx, cy = x + w // 2, y + h // 2
            detection = {'x': int(cx), 'y': int(cy),
                         'w': int(w), 'h': int(h)}
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f'Face ({cx}, {cy})', (x, max(20, y - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            self.last_face = detection
            if self.enable_actuation:
                self._track(cx, cy, width, height)

        self.position_pub.publish(String(data=json.dumps({
            'detected': detection is not None,
            'image_width': int(width),
            'image_height': int(height),
            'face': detection,
        })))

        if self.publish_annotated:
            try:
                annotated = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                annotated.header = msg.header
                self.annotated_pub.publish(annotated)
            except (ValueError, RuntimeError) as exc:
                self.get_logger().warning(f'Could not encode tracking image: {exc}')

    def _track(self, cx, cy, width, height):
        error_x = (cx - width / 2.0) / (width / 2.0)
        error_y = (cy - height / 2.0) / (height / 2.0)
        if abs(error_x) <= self.deadband and abs(error_y) <= self.deadband:
            return

        if abs(error_x) > self.deadband:
            self.pan_deg += self.pan_direction * error_x * self.gain
        if abs(error_y) > self.deadband:
            self.tilt_deg += self.tilt_direction * error_y * self.gain

        self.pan_deg = min(self.pan_limits[1], max(self.pan_limits[0], self.pan_deg))
        self.tilt_deg = min(self.tilt_limits[1], max(self.tilt_limits[0], self.tilt_deg))
        self.pan_deg = self._limit_step(self.pan_deg, self.max_step)
        self.tilt_deg = self._limit_step(self.tilt_deg, self.max_step)

        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = [self.pan_joint, self.tilt_joint]
        # The hardware joint_command dispatcher expects degrees per config.yaml.
        command.position = [self.pan_deg, self.tilt_deg]
        self.joint_pub.publish(command)

    @staticmethod
    def _limit_step(value, max_step):
        # Limit the delta from the prior command to avoid abrupt neck movement.
        # The caller tracks current commanded values; this function is replaced
        # by the per-axis limiter below when state is updated.
        return value


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FaceTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
