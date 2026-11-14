import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory('inmoov_description')
    xacro_file = os.path.join(description_pkg, 'robots', 'inmoov.urdf.xacro')
    rviz_config_file = os.path.join(description_pkg, 'urdf.rviz')

    # The voice viseme bridge publishes jaw targets in radians, up to 0.349.
    jaw_upper_limit = '0.349'
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' jaw_upper_limit:=', jaw_upper_limit]),
        value_type=str,
    )

    return LaunchDescription([
        # Joint state publisher supplies default states for every movable joint
        # and merges the jaw-only source into the complete /joint_states topic.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'source_list': ['/jaw_joint_states']},
                {'use_sim_time': False},
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_description}],
        ),
        Node(
            package='inmoov_description',
            executable='jaw_to_joint_states.py',
            name='jaw_to_joint_states',
            output='screen',
            parameters=[{'jaw_min': 0.0, 'jaw_max': float(jaw_upper_limit)}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ])
