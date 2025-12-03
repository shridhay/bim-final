from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'music_file',
            default_value='/home/user/song.wav',
            description='Path to music file to analyze'
        ),
        DeclareLaunchArgument(
            'joint_index',
            default_value='0',
            description='Joint index to control (0-3)'
        ),
        DeclareLaunchArgument(
            'k_tempo',
            default_value='0.1',
            description='Tempo sensitivity constant'
        ),
        DeclareLaunchArgument(
            'k_energy',
            default_value='0.5',
            description='Energy sensitivity constant'
        ),
        
        # Music analyzer node
        Node(
            package='shutter_music_analyzer',
            executable='music_analyzer',
            name='music_beat_analyzer',
            parameters=[{
                'music_file': LaunchConfiguration('music_file'),
                'publish_interval': 0.1,
            }]
        ),
        
        # Oscillator control node
        Node(
            package='shutter_music_analyzer',
            executable='oscillator_control',
            name='oscillator_control',
            parameters=[{
                'joint_index': LaunchConfiguration('joint_index'),
                'k_tempo': LaunchConfiguration('k_tempo'),
                'k_energy': LaunchConfiguration('k_energy'),
                'base_amplitude': 0.5,
                'control_rate': 50.0,
                'enable_beat_sync': True,
            }]
        ),
    ])

