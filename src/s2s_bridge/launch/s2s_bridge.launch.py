import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('s2s_bridge')
    default_params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS 2 parameters YAML file'
    )

    return LaunchDescription([
        params_file_arg,
        Node(
            package='s2s_bridge',
            executable='viseme_node',
            name='s2s_viseme_bridge',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])

