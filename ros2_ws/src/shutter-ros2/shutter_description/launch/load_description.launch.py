#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
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

    # Get the path to the xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('shutter_description'),
        'robots',
        ['shutter.v.', LaunchConfiguration('robot_version'), '.xacro']
    ])

    # Robot description parameter
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration('run_robot_state_publisher'))
    )

    return LaunchDescription([
        robot_version_arg,
        run_robot_state_publisher_arg,
        robot_state_publisher_node,
    ])
