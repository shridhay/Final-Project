#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Camera framerate'
    )
    
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value=['/dev/v4l/by-id/', EnvironmentVariable('SHUTTER_WEBCAM_ID')],
        description='Video device path'
    )
    
    view_image_arg = DeclareLaunchArgument(
        'view_image',
        default_value='false',
        description='Whether to view the image'
    )

    # USB camera node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='shutter_webcam',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': 1280,
            'image_height': 720,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'camera_color_optical_frame',
            'framerate': LaunchConfiguration('framerate'),
            'autofocus': False,
            'camera_info_url': 'package://shutter_bringup/config/logitech_c920.yaml',
            'camera_name': 'logitech_c920'
        }]
    )

    # Image view node (only if requested)
    image_view_node = Node(
        package='image_view',
        executable='image_view',
        name='image_view',
        remappings=[('/image', '/shutter_webcam/image_raw')],
        parameters=[{'autosize': True}],
        condition=IfCondition(LaunchConfiguration('view_image'))
    )

    return LaunchDescription([
        framerate_arg,
        video_device_arg,
        view_image_arg,
        usb_cam_node,
        image_view_node,
    ])
