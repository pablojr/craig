import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('inmoov_description')
    gz_pkg = get_package_share_directory('craig_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(desc_pkg, 'robots', 'inmoov.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'inmoov', '-topic', 'robot_description'],
        output='screen',
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(gz_pkg, 'config', 'gz_bridge.yaml')}],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        gz_bridge,
    ])

