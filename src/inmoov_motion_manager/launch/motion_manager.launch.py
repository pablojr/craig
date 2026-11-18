from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('inmoov_motion_manager'),
        'config',
        'gestures.yaml',
    )
    return LaunchDescription([
        Node(
            package='inmoov_motion_manager',
            executable='motion_manager',
            name='inmoov_motion_manager',
            output='screen',
            parameters=[{
                'config_file': config,
                'joint_states_topic': '/joint_states',
                'action_topic': '/robot/action',
                'status_topic': '/robot/action/status',
                'publish_rate_hz': 30.0,
            }],
        )
    ])
