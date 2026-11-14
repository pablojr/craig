# USB webcam bring-up (USB/IP + Docker + forwarded RViz)

This setup runs both the webcam driver and RViz in the ROS 2 container on the VM. The webcam is attached to the VM over USB/IP, and RViz is displayed on the laptop through the already-working forwarded X session.

## Device visibility

After USB/IP attach, confirm the VM sees the webcam:

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

The VM host device must also be passed into the Docker container. For a newly created container, include (adjust the path if needed):

```bash
--device=/dev/video0:/dev/video0
```

The user running ROS inside the container needs permission to access the device. If the container already exists without the device mapping, recreate it with the mapping. Confirm visibility inside the container with `ls -l /dev/video*`.

## Install and build

In the Jazzy container:

```bash
sudo apt update
sudo apt install ros-jazzy-usb-cam v4l-utils
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to inmoov_description
source install/setup.bash
```

## Launch camera and RViz

```bash
ros2 launch inmoov_description webcam.launch.py
```

Override the device or capture mode if required by the webcam:

```bash
ros2 launch inmoov_description webcam.launch.py video_device:=/dev/video2 image_width:=1280 image_height:=720 framerate:=30.0
```

The default image topic is `/camera/image_raw`; camera calibration information is published on `/camera/camera_info`. Confirm publishing in another sourced shell:

```bash
ros2 topic list | grep '^/camera/'
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/camera_info
```

Supported resolution, frame rate, and pixel formats depend on the webcam. Inspect them on the VM (or in the container if the utility is installed):

```bash
v4l2-ctl --list-formats-ext -d /dev/video0
```

If RViz opens but reports no image, check topic publishing, confirm the Image display uses `/camera/image_raw`, and inspect camera-node logs for permissions or unsupported capture modes. RViz uses the forwarded X display, as in the jaw visualization setup.
