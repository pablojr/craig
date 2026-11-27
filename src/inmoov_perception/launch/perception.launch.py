import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Camera node (v4l2_camera)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        output='screen',
        parameters=[
            {'video_device': '/dev/video0'},
            {'image_size': [640, 480]},
            {'time_per_frame': [1, 30]},
        ],
        remappings=[('image_raw', '/camera/image_raw')],
    )

    # Perception node
    perception_node = Node(
        package='inmoov_perception',
        executable='inmoov_perception_node',
        name='inmoov_perception_node',
        output='screen',
    )

    return LaunchDescription([
        camera_node,
        perception_node,
    ])
