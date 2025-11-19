#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    robot_version_arg = DeclareLaunchArgument(
        'robot_version',
        default_value='2.0',
        description='Robot version to use'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Set to true when running Unity simulation without a display'
    )

    # Include the main shutter launch file with simulation enabled
    shutter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('shutter_bringup'),
                'launch',
                'shutter.launch.py'
            ])
        ]),
        launch_arguments={
            'simulation': 'true',
            'robot_version': LaunchConfiguration('robot_version'),
            'headless': LaunchConfiguration('headless')
        }.items()
    )

    return LaunchDescription([
        robot_version_arg,
        headless_arg,
        shutter_launch,
    ])
