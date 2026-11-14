import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory('inmoov_description')
    xacro_file = os.path.join(description_pkg, 'robots', 'inmoov.urdf.xacro')
    merger_script = os.path.join(description_pkg, 'jaw_state_merger.py')
    rviz_config_file = os.path.join(description_pkg, 'urdf.rviz')

    # The voice viseme bridge publishes jaw targets in radians, up to 0.349.
    jaw_upper_limit = '0.349'
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' jaw_upper_limit:=', jaw_upper_limit]),
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
        # Execute with Python explicitly; this does not depend on a ROS
        # joint_state_publisher package or executable bit on the installed file.
        ExecuteProcess(
            cmd=['python3', merger_script],
            name='jaw_state_merger',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ])
