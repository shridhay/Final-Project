#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Whether to run in simulation mode'
    )
    
    display_id_arg = DeclareLaunchArgument(
        'display_id',
        default_value='1011',
        description='Display ID for face rendering'
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
    
    blink_arg = DeclareLaunchArgument(
        'blink',
        default_value='true',
        description='Whether to enable blinking'
    )
    
    limit_pupils_arg = DeclareLaunchArgument(
        'limit_pupils',
        default_value='true',
        description='Whether to limit pupil movement'
    )
    
    move_to_shutter_screen_arg = DeclareLaunchArgument(
        'move_to_shutter_screen',
        default_value='true',
        description='Whether to move to shutter screen'
    )

    # Determine screen version based on robot version
    screen_version = PythonExpression([
        '"v2" if float("', LaunchConfiguration('robot_version'), '") >= 4.0 else "v1"'
    ])

    # Include the main shutter launch file
    shutter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('shutter_bringup'),
                'launch',
                'shutter.launch.py'
            ])
        ]),
        launch_arguments={
            'simulation': LaunchConfiguration('simulation'),
            'robot_version': LaunchConfiguration('robot_version'),
            'run_robot_state_publisher': LaunchConfiguration('run_robot_state_publisher'),
            'driver_device': LaunchConfiguration('driver_device'),
            'run_rosbridge': LaunchConfiguration('run_rosbridge')
        }.items()
    )

    # Include the face launch file
    face_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('shutter_face_ros'),
                'launch',
                'simple_face.launch.py'
            ])
        ]),
        launch_arguments={
            'blink': LaunchConfiguration('blink'),
            'limit_pupils': LaunchConfiguration('limit_pupils'),
            'move_to_shutter_screen': LaunchConfiguration('move_to_shutter_screen'),
            'screen_version': screen_version,
            'simulation': LaunchConfiguration('simulation'),
            'display_id': LaunchConfiguration('display_id')
        }.items()
    )

    motor_start_node = Node(
        package='shutter_bringup',
        executable='motor_startup_publisher.py',
        output='screen'
    )

    return LaunchDescription([
        simulation_arg,
        display_id_arg,
        robot_version_arg,
        run_robot_state_publisher_arg,
        driver_device_arg,
        run_rosbridge_arg,
        blink_arg,
        limit_pupils_arg,
        move_to_shutter_screen_arg,
        shutter_launch,
        face_launch,
        motor_start_node, 
    ])
