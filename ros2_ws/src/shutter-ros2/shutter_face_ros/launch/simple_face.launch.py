#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    blink_arg = DeclareLaunchArgument(
        'blink',
        default_value='true',
        description='Enable blinking'
    )
    
    blink_playback_arg = DeclareLaunchArgument(
        'blink_playback',
        default_value='false',
        description='Use playback blinking instead of generated blinking'
    )
    
    limit_pupils_arg = DeclareLaunchArgument(
        'limit_pupils',
        default_value='true',
        description='Limit pupil position within eye boundaries'
    )
    
    move_to_shutter_screen_arg = DeclareLaunchArgument(
        'move_to_shutter_screen',
        default_value='true',
        description='Move face window to shutter screen'
    )
    
    screen_version_arg = DeclareLaunchArgument(
        'screen_version',
        default_value='v1',
        description='Screen version (v1 or v2)'
    )
    
    fx_arg = DeclareLaunchArgument(
        'fx',
        default_value='40',
        description='Focal length x parameter'
    )
    
    fy_arg = DeclareLaunchArgument(
        'fy',
        default_value='40',
        description='Focal length y parameter'
    )
    
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Run in simulation mode'
    )
    
    display_id_arg = DeclareLaunchArgument(
        'display_id',
        default_value='1011',
        description='X display ID for xvfb'
    )

    # Derived arguments
    xauth = PathJoinSubstitution([
        FindPackageShare('shutter_face_ros'),
        '.xvfb_cookie'
    ])
    
    # Screen size selection based on screen_version
    xscreen_size = PythonExpression([
        "'800x480x24' if '", LaunchConfiguration('screen_version'), "' == 'v1' else '1024x600x24'"
    ])
    
    # xvfb prefix: empty unless simulation is true
    # This matches the ROS 1 behavior: empty prefix unless simulation, then xvfb-run command
    xvfb_prefix = PythonExpression([
        "'' if '", LaunchConfiguration('simulation'), "' != 'true' else ",
        "'xvfb-run -n ' + '", LaunchConfiguration('display_id'), 
        "' + ' -f ' + '", xauth, 
        "' + ' --server-args=\\'-screen 0 ' + '", xscreen_size, "' + '\\''"
    ])

    # Simple face node
    simple_face_node = Node(
        package='shutter_face_ros',
        executable='simple_face.py',
        name='simple_face',
        output='screen',
        prefix=xvfb_prefix,
        parameters=[{
            'blink': LaunchConfiguration('blink'),
            'blink_playback': LaunchConfiguration('blink_playback'),
            'limit_pupils': LaunchConfiguration('limit_pupils'),
            'move_to_shutter_screen': LaunchConfiguration('move_to_shutter_screen'),
            'screen_version': LaunchConfiguration('screen_version'),
        }]
    )

    # Gaze master node
    gaze_master_node = Node(
        package='shutter_face_ros',
        executable='gaze_master.py',
        name='gaze_master',
        output='screen',
        parameters=[{
            'fx': LaunchConfiguration('fx'),
            'fy': LaunchConfiguration('fy'),
        }]
    )

    # Screen grab include (conditional, only in simulation mode)
    # Note: In ROS2, remappings must be applied at the Node level or passed as launch arguments.
    # If remapping /image to /virtual_face/image is needed, the screen_grab launch file
    # should support it via launch arguments, or nodes should be created with remappings directly.
    screen_grab_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('screen_grab'),
                'launch',
                'screen_grab.launch.py'
            ])
        ]),
        launch_arguments={
            'width': '0',
            'height': '0',
            'display_id': LaunchConfiguration('display_id'),
            'sleep': '5',
        }.items(),
        condition=IfCondition(LaunchConfiguration('simulation'))
    )

    # Set XAUTHORITY environment variable (conditional, only in simulation mode)
    set_xauth_env = SetEnvironmentVariable(
        'XAUTHORITY',
        xauth,
        condition=IfCondition(LaunchConfiguration('simulation'))
    )

    return LaunchDescription([
        blink_arg,
        blink_playback_arg,
        limit_pupils_arg,
        move_to_shutter_screen_arg,
        screen_version_arg,
        fx_arg,
        fy_arg,
        simulation_arg,
        display_id_arg,
        set_xauth_env,
        simple_face_node,
        gaze_master_node,
        screen_grab_launch,
    ])
