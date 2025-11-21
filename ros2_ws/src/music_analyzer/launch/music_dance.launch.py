from launch import LaunchDescription
from launch_ros.actions import Node

'''
NOTE: NOT NEEDED ANYMORE, THIS WAS FOR REAL-TIME AUDIO CAPTURE TESTING, but running into package installation issues so no longer doing realtime updates
'''

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='audio_common',
            executable='audio_capture',
            name='audio_capture',
            parameters=[
                {'device': 'default'},
                {'channels': 1},
                {'depth': 16},
                {'sample_rate': 44100},
            ]
        ),
        Node(
            package='music_analyzer',
            executable='music_analyzer',
            name='music_analyzer',
            parameters=[
                {'audio_topic': '/audio'},
                {'sample_rate': 44100},
            ]
        ),
    ])