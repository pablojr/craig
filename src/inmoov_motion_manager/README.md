# InMoov Motion Manager (RViz first step)

A small ROS 2 Python package for testing predefined InMoov gestures in RViz.
It publishes `sensor_msgs/JointState`; it does **not** command motors or
simulate dynamics.

## Important before running

`config/gestures.yaml` contains illustrative joint names and example angles.
They are not verified against your InMoov URDF. Replace the names with the
exact URDF joint names and adjust `home`, `min`, `max`, and gesture positions
to the joint axes and limits in your robot description.

Do not run the example trajectories unchanged against hardware.

The existing `/jaw` and `jaw_to_joint_states` path is left unchanged. This
package does not include `jaw_joint` in its messages.

## Interfaces

- Subscribe: `/robot/action` (`std_msgs/msg/String`)
- Publish: `/robot/action/status` (`std_msgs/msg/String`, JSON payload)
- Publish: `/joint_states` (`sensor_msgs/msg/JointState`)

Action JSON examples:

```json
{"action":"raise_hand","side":"left"}
{"action":"wave","side":"left"}
{"action":"nod","direction":"yes"}
```

The manager rejects a new action while another gesture is running. It reports
`accepted`, `rejected`, and `completed` status messages. This initial version
does not implement cancellation or gesture preemption.

## Build

Copy this package directory into your ROS 2 workspace `src` directory:

```bash
cd ~/ros2_ws/src
# copy/extract inmoov_motion_manager here

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select inmoov_motion_manager
source install/setup.bash
```

If `rosdep` does not install PyYAML, install the ROS distribution's
`python3-yaml` package using your system package manager.

## Configure

Edit:

```text
src/inmoov_motion_manager/config/gestures.yaml
```

For every joint used by a gesture:
1. Use the exact joint name from the URDF.
2. Set a safe `home` position.
3. Set the URDF's valid lower and upper limits.
4. Replace example gesture positions with values that move in the intended
   direction and remain within limits.

Rebuild after editing if using the installed package copy:

```bash
colcon build --packages-select inmoov_motion_manager
source install/setup.bash
```

## Run

```bash
ros2 launch inmoov_motion_manager motion_manager.launch.py
```

Check the interfaces:

```bash
ros2 topic info /robot/action
ros2 topic info /joint_states
ros2 topic echo /robot/action/status
```

## Test gestures

In another sourced terminal:

```bash
ros2 topic pub --once /robot/action std_msgs/msg/String   "{data: '{"action":"raise_hand","side":"left"}'}"

ros2 topic pub --once /robot/action std_msgs/msg/String   "{data: '{"action":"wave","side":"left"}'}"

ros2 topic pub --once /robot/action std_msgs/msg/String   "{data: '{"action":"nod","direction":"yes"}'}"
```

Open RViz with your existing InMoov display configuration and verify the
movement. The node publishes all joints configured in `gestures.yaml` on each
update, so avoid configuring the same joint in another independent publisher.

## Current limitations

- RViz visualization only; no motor, collision, inertia, or dynamics control.
- Only one gesture can run at a time.
- No action cancellation or safety-rated stop yet.
- No automatic return to home after a gesture; gesture frames must include
  return-to-home positions if desired.
- Gesture joint names and angles require validation against the actual URDF.
