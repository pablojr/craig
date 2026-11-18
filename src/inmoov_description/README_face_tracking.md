# USB webcam and face tracking

The webcam launch runs the existing `usb_cam` camera publisher, the face tracker,
and RViz together. Face detection uses a Haar cascade with CLAHE, adapted from
[PYF07/face_tracker](https://github.com/PYF07/face_tracker), but subscribes to
`/camera/image_raw` rather than reopening the camera device. RViz displays the
annotated `/camera/tracking_image` stream.

## Build and run (ROS 2 Jazzy)

Install dependencies if needed:

```bash
sudo apt update
sudo apt install ros-jazzy-usb-cam ros-jazzy-cv-bridge python3-opencv
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to inmoov_description
source install/setup.bash
```

Start detection and the camera visualization:

```bash
ros2 launch inmoov_description webcam.launch.py
```

This detects and publishes face coordinates, but **neck movement is disabled by
default**. Once the robot is clear and the direction/limits have been checked,
explicitly enable hardware commands:

```bash
ros2 launch inmoov_description webcam.launch.py enable_actuation:=true
```

The tracker publishes degree-valued `JointState` commands for
`head_pan_joint` and `head_tilt_joint` on `/joint_command`, matching the current
hardware dispatcher/config convention. The node clamps its targets to the
configured defaults (-100..90 degrees pan, -20..20 degrees tilt), applies a
center deadband, and limits each update. Mechanical travel, sign, and calibration
vary: keep actuation disabled until checked, and adjust `pan_direction` or
`tilt_direction` (1 or -1) as needed. Stop the launch immediately if motion is
unexpected. This is a basic visual servo, not a safety-rated controller.

## Topics and controls

- Input: `/camera/image_raw` (`sensor_msgs/msg/Image`)
- Annotated view: `/camera/tracking_image` (`sensor_msgs/msg/Image`)
- Detection: `/face_position` (`std_msgs/msg/String`, JSON; `face` is null when
  no face is found)
- Neck command when enabled: `/joint_command`

Useful checks:

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /face_position
ros2 topic echo /joint_command
```

`launch_tracker:=false` disables tracking while retaining the camera and RViz.
