import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory('inmoov_description')
    rviz_config_file = os.path.join(description_pkg, 'config', 'webcam.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='Video device visible inside the ROS 2 container'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Launch RViz in the VM X-forwarded display'),
        DeclareLaunchArgument(
            'launch_tracker', default_value='true',
            description='Run face detection on /camera/image_raw'),
        DeclareLaunchArgument(
            'enable_actuation', default_value='false',
            description='Explicitly enable InMoov neck movement'),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='camera',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'image_width': ParameterValue(LaunchConfiguration('image_width'), value_type=int),
                'image_height': ParameterValue(LaunchConfiguration('image_height'), value_type=int),
                'framerate': ParameterValue(LaunchConfiguration('framerate'), value_type=float),
                'camera_name': 'webcam',
                'frame_id': 'camera_optical_frame',
                'io_method': 'mmap',
            }],
        ),
        Node(
            package='inmoov_description',
            executable='face_tracker_node.py',
            name='face_tracker',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'enable_actuation': ParameterValue(
                    LaunchConfiguration('enable_actuation'), value_type=bool),
            }],
            condition=IfCondition(LaunchConfiguration('launch_tracker')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='webcam_rviz',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
        ),
    ])
