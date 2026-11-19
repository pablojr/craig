import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory('inmoov_description')
    motion_manager_pkg = get_package_share_directory('inmoov_motion_manager')
    xacro_file = os.path.join(description_pkg, 'robots', 'inmoov.urdf.xacro')
    gestures_file = os.path.join(motion_manager_pkg, 'config', 'gestures.yaml')
    rviz_config_file = os.path.join(description_pkg, 'urdf.rviz')

    # Keep the RViz jaw travel wide enough for the viseme bridge's /jaw targets.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' jaw_upper_limit:=0.349']),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_description}],
        ),
        # MotionManager is the sole /joint_states publisher in this RViz launch.
        # It also incorporates /jaw targets, replacing jaw_state_merger here.
        Node(
            package='inmoov_motion_manager',
            executable='motion_manager',
            name='inmoov_motion_manager',
            output='screen',
            parameters=[{
                'config_file': gestures_file,
                'joint_states_topic': '/joint_states',
                'action_topic': '/robot/action',
                'status_topic': '/robot/action/status',
                'publish_rate_hz': 30.0,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ])
