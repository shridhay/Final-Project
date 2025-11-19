#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Whether to run in simulation mode'
    )
    
    robot_version_arg = DeclareLaunchArgument(
        'robot_version',
        default_value='2.0',
        description='Robot version to use'
    )
    
    run_robot_state_publisher_arg = DeclareLaunchArgument(
        'run_robot_state_publisher',
        default_value='true',
        description='Whether to run robot state publisher'
    )
    
    driver_device_arg = DeclareLaunchArgument(
        'driver_device',
        default_value='ttyUSB0',
        description='Driver device name'
    )
    
    run_rosbridge_arg = DeclareLaunchArgument(
        'run_rosbridge',
        default_value='false',
        description='Whether to run rosbridge'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Set to true when running Unity simulation without a display'
    )

    # Load the robot model
    load_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('shutter_description'),
                'launch',
                'load_description.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_version': LaunchConfiguration('robot_version'),
            'run_robot_state_publisher': LaunchConfiguration('run_robot_state_publisher')
        }.items()
    )

    # Hardware interface (only if not simulation)
    hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('shutter_hardware_interface'),
                'launch',
                'shutter_position_interface.launch.py'
            ])
        ]),
        launch_arguments={
            'usb_port': ['/dev/', LaunchConfiguration('driver_device')]
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('simulation'))
    )

    # Unity simulation (only if simulation)
    unity_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('shutter_bringup'),
                'launch',
                'shutter_unity.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('simulation'))
    )

    # Rosbridge (only if requested)
    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket.launch.py'
            ])
        ]),
        launch_arguments={
            'port': '5555',
            'topics_glob': '[/joint_*/command]',
            'services_glob': '',
            'params_glob': ''
        }.items(),
        condition=IfCondition(LaunchConfiguration('run_rosbridge'))
    )

    return LaunchDescription([
        simulation_arg,
        robot_version_arg,
        run_robot_state_publisher_arg,
        driver_device_arg,
        run_rosbridge_arg,
        headless_arg,
        load_description_launch,
        hardware_interface_launch,
        unity_simulation_launch,
        rosbridge_launch,
    ])
