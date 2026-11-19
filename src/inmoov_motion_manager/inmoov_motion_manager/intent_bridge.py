#!/usr/bin/env python3
"""Translate structured, extracted robot intents into motion-manager actions.

This node deliberately does not perform language understanding. An upstream
intent extractor publishes JSON on /robot/intent; ordinary dialogue can stay
on the speech pipeline and is ignored unless it produces an allowlisted intent.
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IntentBridge(Node):
    def __init__(self):
        super().__init__('inmoov_intent_bridge')
        self.declare_parameter('intent_topic', '/robot/intent')
        self.declare_parameter('action_topic', '/robot/action')
        self.declare_parameter('action_status_topic', '/robot/action/status')
        self.declare_parameter('minimum_confidence', 0.80)
        self.declare_parameter('max_sequence_length', 4)

        self.minimum_confidence = float(self.get_parameter('minimum_confidence').value)
        self.max_sequence_length = int(self.get_parameter('max_sequence_length').value)
        self.action_pub = self.create_publisher(
            String, self.get_parameter('action_topic').value, 10
        )
        self.intent_sub = self.create_subscription(
            String, self.get_parameter('intent_topic').value, self._on_intent, 10
        )
        self.status_sub = self.create_subscription(
            String, self.get_parameter('action_status_topic').value,
            self._on_action_status, 10,
        )
        self._pending_actions = []
        self._active_action = None
        self.get_logger().info(
            f'Listening for structured intents on '
            f'{self.get_parameter("intent_topic").value}; '
            f'minimum confidence={self.minimum_confidence:.2f}'
        )

    @staticmethod
    def _simple_action(intent, request):
        """Return a safe motion-manager action, or None for unknown intents."""
        if intent in ('nod', 'nod_yes', 'agree'):
            return {'action': 'nod', 'direction': 'yes'}

        if intent in ('raise_hand', 'wave', 'lower_hand'):
            side = str(request.get('side', '')).strip().lower()
            if side not in ('left', 'right'):
                raise ValueError(f'{intent} requires side="left" or side="right"')
            if intent == 'lower_hand':
                return {'action': 'lower_hand', 'side': side}
            return {'action': intent, 'side': side}

        return None

    def _expand_intent(self, request):
        if not isinstance(request, dict):
            raise ValueError('Intent payload must be a JSON object')
        intent = str(request.get('intent', '')).strip().lower()

        # Explicit non-robot/ordinary-dialogue intents are valid no-ops.
        if intent in ('', 'none', 'ordinary_dialogue', 'no_robot_action'):
            return []

        if intent == 'sequence':
            commands = request.get('commands')
            if not isinstance(commands, list) or not commands:
                raise ValueError('sequence intent requires a non-empty commands list')
            if len(commands) > self.max_sequence_length:
                raise ValueError(
                    f'sequence exceeds max_sequence_length={self.max_sequence_length}'
                )
            expanded = []
            for command in commands:
                if not isinstance(command, dict):
                    raise ValueError('Each sequence command must be an object')
                command_intent = str(command.get('intent', '')).strip().lower()
                action = self._simple_action(command_intent, command)
                if action is None:
                    raise ValueError(f'Unsupported sequence intent: {command_intent}')
                expanded.append(action)
            return expanded

        if intent in ('say_hello', 'greet', 'wave_goodbye'):
            side = str(request.get('side', 'both')).strip().lower()
            if side == 'both':
                return [
                    {'action': 'wave', 'side': 'left'},
                    {'action': 'wave', 'side': 'right'},
                ]
            if side not in ('left', 'right'):
                raise ValueError('side must be left, right, or both')
            return [{'action': 'wave', 'side': side}]

        action = self._simple_action(intent, request)
        if action is None:
            self.get_logger().info(f'Ignoring unrecognized robot intent: {intent!r}')
            return []
        return [action]

    def _on_intent(self, msg):
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict):
                raise ValueError('Intent payload must be a JSON object')
            confidence = float(request.get('confidence', 1.0))
            if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
                raise ValueError('confidence must be a finite number in [0, 1]')
            if confidence < self.minimum_confidence:
                self.get_logger().info(
                    f'Ignoring low-confidence intent ({confidence:.2f}): '
                    f'{request.get("intent", "")}'
                )
                return

            actions = self._expand_intent(request)
            if not actions:
                return
            if self._active_action is not None or self._pending_actions:
                self.get_logger().warning('Ignoring intent; a gesture sequence is active')
                return
            self._pending_actions = actions
            self._publish_next_action()
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f'Ignoring invalid robot intent: {exc}')

    def _publish_next_action(self):
        if not self._pending_actions:
            self._active_action = None
            return
        self._active_action = self._pending_actions.pop(0)
        msg = String()
        msg.data = json.dumps(self._active_action)
        self.action_pub.publish(msg)
        self.get_logger().info(f'Forwarded robot action: {msg.data}')

    def _on_action_status(self, msg):
        if self._active_action is None:
            return
        try:
            status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning('Ignoring malformed motion-manager status')
            return

        if status.get('action') != self._active_action.get('action'):
            return
        if status.get('state') == 'completed':
            self._publish_next_action()
        elif status.get('state') == 'rejected':
            self.get_logger().warning(
                f'Gesture sequence stopped: {status.get("message", "action rejected")}'
            )
            self._pending_actions.clear()
            self._active_action = None


def main(args=None):
    rclpy.init(args=args)
    node = IntentBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
