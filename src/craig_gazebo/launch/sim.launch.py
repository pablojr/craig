import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('inmoov_description')
    meshes_pkg = get_package_share_directory('inmoov_meshes')
    gz_pkg = get_package_share_directory('craig_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # Gazebo resolves model://inmoov_meshes/... by searching parent folders in GZ_SIM_RESOURCE_PATH
    meshes_parent_dir = os.path.dirname(meshes_pkg)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = (
        f'{meshes_parent_dir}:{existing_gz_path}' if existing_gz_path else meshes_parent_dir
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path,
    )

    xacro_file = os.path.join(desc_pkg, 'robots', 'inmoov.urdf.xacro')
    # Widen only the simulation model jaw limit to make the viseme motion visible.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' jaw_upper_limit:=0.349']),
        value_type=str,
    )

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
        arguments=[
            '-name', 'inmoov',
            '-topic', 'robot_description',
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
        output='screen',
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(gz_pkg, 'config', 'gz_bridge.yaml')}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='3.14159',
            description='Initial InMoov yaw in radians; default faces the robot toward the camera.',
        ),
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        gz_bridge,
    ])
