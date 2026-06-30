import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    robotXacroName = 'differential_drive_robot'
    packageName = 'mobile_robot'

    # Robot xacro path
    modelFilePath = os.path.join(
        get_package_share_directory(packageName),
        'model',
        'robot.xacro'
    )

    # World path
    worldFilePath = os.path.join(
        get_package_share_directory(packageName),
        'world',
        'my_world.sdf'
    )

    # Process xacro
    robotDescription = xacro.process_file(modelFilePath).toxml()

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {worldFilePath}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.2'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Bridge config
    bridge_params = os.path.join(
        get_package_share_directory(packageName),
        'parameters',
        'bridge_parameters.yaml'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ],
        output='screen'
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'differential_drive_robot/base_footprint/gpu_lidar'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        static_tf
    ])