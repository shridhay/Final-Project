#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('shutter_hardware_interface')

    usb_port_arg  = DeclareLaunchArgument('usb_port',  default_value='/dev/ttyUSB0')
    baud_rate_arg = DeclareLaunchArgument('baud_rate', default_value='4000000')
    use_js_arg    = DeclareLaunchArgument('use_joint_states_topic', default_value='true')

    dynamixel_info = PathJoinSubstitution([pkg_share, 'config', 'dynamixel_joints_position.yaml'])

    node = Node(
        package='shutter_hardware_interface',
        executable='shutter_position_node',
        name='shutter_position_interface',
        output='screen',
        parameters=[{
            'port_name': LaunchConfiguration('usb_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'dynamixel_info': dynamixel_info,            # string path, not a list
            'use_joint_states_topic': LaunchConfiguration('use_joint_states_topic'),
        }],
    )

    return LaunchDescription([
        usb_port_arg,
        baud_rate_arg,
        use_js_arg,
        node,
    ])


