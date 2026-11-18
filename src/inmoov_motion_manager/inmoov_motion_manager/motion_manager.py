#!/usr/bin/env python3
"""Execute configured InMoov gestures by publishing JointState messages.

Command interface:
  /robot/action        std_msgs/String containing JSON, e.g.
                       {"action":"raise_hand","side":"left"}
  /robot/action/status std_msgs/String containing JSON status updates

This node is intended for RViz visualization. It does not command motors.
"""

import json
import math
import threading
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class MotionManager(Node):
    def __init__(self):
        super().__init__('inmoov_motion_manager')

        default_config = str(
            Path(get_package_share_directory('inmoov_motion_manager'))
            / 'config' / 'gestures.yaml'
        )
        self.declare_parameter('config_file', default_config)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('action_topic', '/robot/action')
        self.declare_parameter('status_topic', '/robot/action/status')
        self.declare_parameter('publish_rate_hz', 30.0)

        config_path = self.get_parameter('config_file').value
        self.publish_rate_hz = max(
            5.0, float(self.get_parameter('publish_rate_hz').value)
        )
        self.dt = 1.0 / self.publish_rate_hz

        self.joint_pub = self.create_publisher(
            JointState, self.get_parameter('joint_states_topic').value, 10
        )
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10
        )
        self.action_sub = self.create_subscription(
            String, self.get_parameter('action_topic').value,
            self._on_action, 10
        )

        with open(config_path, 'r', encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream) or {}

        self.joints = self.config.get('joints', {})
        self.gestures = self.config.get('gestures', {})
        self.joint_names = list(self.joints.keys())
        self.positions = {
            name: float(spec.get('home', 0.0))
            for name, spec in self.joints.items()
        }
        self.limits = {}
        for name, spec in self.joints.items():
            lower = float(spec.get('min', -math.pi))
            upper = float(spec.get('max', math.pi))
            if lower > upper:
                raise ValueError(f'Invalid limits for {name}: min > max')
            self.limits[name] = (lower, upper)
            self.positions[name] = self._clamp(name, self.positions[name])

        self._lock = threading.Lock()
        self._active = False
        self._frames = []
        self._frame_index = 0
        self._frame_elapsed = 0.0
        self._frame_start = {}
        self._goal = {}
        self._current_action = None

        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'Loaded {len(self.joint_names)} joints and '
            f'{len(self.gestures)} gestures from {config_path}'
        )
        if not self.joint_names:
            self.get_logger().warning(
                'No joints configured. Edit gestures.yaml before testing gestures.'
            )
        self._publish_joint_state()

    def _clamp(self, joint, value):
        lower, upper = self.limits[joint]
        return min(upper, max(lower, float(value)))

    def _publish_status(self, state, action, message=''):
        payload = {
            'state': state,
            'action': action,
            'message': message,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def _publish_joint_state(self):
        if not self.joint_names:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.positions[name] for name in self.joint_names]
        self.joint_pub.publish(msg)

    def _on_action(self, msg):
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict):
                raise ValueError('Request must be a JSON object')
            action = str(request.get('action', '')).strip()
            if not action:
                raise ValueError('Missing "action"')
            params = request.get('parameters', {})
            if not isinstance(params, dict):
                raise ValueError('"parameters" must be an object')

            # Support the simple {"action":"raise_hand","side":"left"} form.
            params = {**request, **params}
            gesture_name = self._resolve_gesture(action, params)
            if gesture_name not in self.gestures:
                raise ValueError(f'Unknown or unconfigured gesture: {gesture_name}')

            with self._lock:
                if self._active:
                    self._publish_status(
                        'rejected', action, 'Another gesture is currently running'
                    )
                    return
                frames = self.gestures[gesture_name].get('frames', [])
                if not frames:
                    raise ValueError(f'Gesture {gesture_name} has no frames')
                self._validate_frames(frames)
                self._frames = frames
                self._frame_index = 0
                self._frame_elapsed = 0.0
                self._frame_start = dict(self.positions)
                self._goal = self._frame_positions(frames[0])
                self._current_action = action
                self._active = True

            self._publish_status('accepted', action, f'Executing {gesture_name}')
        except Exception as exc:
            self.get_logger().warning(f'Rejected action request: {exc}')
            self._publish_status('rejected', 'unknown', str(exc))

    @staticmethod
    def _resolve_gesture(action, params):
        if action == 'raise_hand':
            return f"raise_{params.get('side', 'left')}_hand"
        if action == 'wave':
            return f"wave_{params.get('side', 'left')}"
        if action == 'nod':
            return 'nod_yes' if params.get('direction', 'yes') == 'yes' else 'nod_yes'
        return action

    def _frame_positions(self, frame):
        requested = frame.get('positions', {})
        result = {}
        for joint, value in requested.items():
            if joint not in self.joints:
                raise ValueError(f'Gesture references unconfigured joint: {joint}')
            result[joint] = self._clamp(joint, value)
        return result

    def _validate_frames(self, frames):
        for index, frame in enumerate(frames):
            duration = float(frame.get('duration', 0.0))
            if not math.isfinite(duration) or duration <= 0.0:
                raise ValueError(f'Frame {index} duration must be positive')
            self._frame_positions(frame)

    def _tick(self):
        with self._lock:
            if not self._active:
                return

            frame = self._frames[self._frame_index]
            duration = float(frame['duration'])
            self._frame_elapsed = min(duration, self._frame_elapsed + self.dt)
            ratio = self._frame_elapsed / duration

            for joint, target in self._goal.items():
                start = self._frame_start.get(joint, self.positions[joint])
                self.positions[joint] = start + (target - start) * ratio

            self._publish_joint_state()

            if self._frame_elapsed >= duration:
                self._frame_index += 1
                if self._frame_index >= len(self._frames):
                    action = self._current_action
                    self._active = False
                    self._current_action = None
                    self._publish_status('completed', action)
                    return

                self._frame_elapsed = 0.0
                self._frame_start = dict(self.positions)
                self._goal = self._frame_positions(
                    self._frames[self._frame_index]
                )

    def destroy_node(self):
        self.timer.cancel()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
