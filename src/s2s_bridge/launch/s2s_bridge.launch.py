from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='s2s_bridge',
            executable='viseme_node',
            name='s2s_viseme_bridge',
            output='screen',
            parameters=[{
                'server_url': 'ws://192.168.0.202:8765/v1/realtime'
            }]
        )
    ])
