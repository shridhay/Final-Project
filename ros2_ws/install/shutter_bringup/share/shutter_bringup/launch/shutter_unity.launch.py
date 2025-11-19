#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Whether to run in debug mode'
    )

    # Simulation manager node
    simulation_manager_node = Node(
        package='shutter_bringup',
        executable='unity_simulation_manager.py',
        name='simulation_manager',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('debug'))
    )

    # Unity simulation node
    unity_simulation_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='unity_simulation',
        arguments=['--wait'],
        output='screen',
        respawn=True
    )

    return LaunchDescription([
        debug_arg,
        simulation_manager_node,
        unity_simulation_node,
    ])
