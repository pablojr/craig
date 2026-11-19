# InMoov Motion Manager and Intent Bridge

This ROS 2 package publishes configured `JointState` gestures for RViz and
provides a separate, deliberately constrained integration node for structured
robot intents. Neither node commands hardware.

## Separation from conversation

`intent_bridge` does not perform NLP and does not connect to or alter the speech
server. An upstream conversation/intent extractor publishes a structured event
to `/robot/intent` (`std_msgs/msg/String`). Ordinary dialogue should produce
`{"intent":"none"}` or no intent event and is never forwarded to the robot.
Only recognized, allowlisted intents with confidence at or above 0.80 become
`/robot/action` requests. The conversation pipeline remains responsible for
answering normally; for an affirmative response such as “Yes, clouds are
white,” it can emit a `nod_yes` event alongside its normal response.

### Input examples

```json
{"intent":"nod_yes","confidence":0.96}
{"intent":"raise_hand","side":"left","confidence":0.99}
{"intent":"say_hello","confidence":0.94}
{"intent":"nod","confidence":0.93}
{"intent":"wave_goodbye","confidence":0.92}
{"intent":"lower_hand","side":"left","confidence":0.98}
{"intent":"sequence","commands":[{"intent":"raise_hand","side":"right"},{"intent":"wave","side":"right"}],"confidence":0.97}
{"intent":"none","confidence":0.99}
```

`say_hello` and `wave_goodbye` wave left then right by default; set `side` to
`left` or `right` for a one-sided wave. Sequences are limited to four
allowlisted actions and execute one at a time using motion-manager completion
status. Invalid, low-confidence, unknown, or concurrent intents are ignored;
ordinary text is not parsed as a robot command in this node.

The intent producer must publish only after its own intent extraction/decision.
For agreement nods, emit `nod_yes` only when the assistant is actually choosing
to agree, not merely because the user asked a question. This keeps inference
outside the ROS robot-control boundary and avoids keyword-triggered movement.

## Interfaces

- Subscribe: `/robot/intent` (`std_msgs/msg/String`, structured JSON above)
- Publish: `/robot/action` (`std_msgs/msg/String`, consumed by motion manager)
- Subscribe: `/robot/action/status` for sequencing
- Publish: `/joint_states` (`sensor_msgs/msg/JointState`)
- Subscribe: `/jaw` (`std_msgs/msg/Float64`)

## RViz

`inmoov_description display.launch.py` starts the robot description,
motion manager, intent bridge, and RViz. Publish an extracted intent for a
test:

```bash
ros2 topic pub --once /robot/intent std_msgs/msg/String \
  "{data: '{\"intent\":\"raise_hand\",\"side\":\"left\",\"confidence\":0.99}'}"
```

Manual action testing remains available on `/robot/action`:

```bash
ros2 topic pub --once /robot/action std_msgs/msg/String \
  "{data: '{\"action\":\"nod\",\"direction\":\"yes\"}'}"
```

Build and launch:

```bash
cd ~/ros2_ws
colcon build --packages-up-to inmoov_description --symlink-install
source install/setup.bash
ros2 launch inmoov_description display.launch.py
```

## Safety note

These are visualization gestures only. Do not connect this path to motor
control without separate limits, authorization, emergency-stop, and hardware
validation.
