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
    
    # Music and oscillator arguments
    use_music_arg = DeclareLaunchArgument(
        'use_music',
        default_value='false',
        description='Enable music-driven motion'
    )
    
    music_file_arg = DeclareLaunchArgument(
        'music_file',
        default_value='/home/user/song.wav',
        description='Path to music file for music-driven motion'
    )
    
    k_tempo_arg = DeclareLaunchArgument(
        'k_tempo',
        default_value='0.1',
        description='Tempo sensitivity constant for oscillator'
    )
    
    k_energy_arg = DeclareLaunchArgument(
        'k_energy',
        default_value='0.5',
        description='Energy sensitivity constant for oscillator'
    )
    
    base_amplitude_arg = DeclareLaunchArgument(
        'base_amplitude',
        default_value='0.5',
        description='Base amplitude A_0 for oscillator'
    )
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='50.0',
        description='Control loop frequency in Hz'
    )
    
    enable_beat_sync_arg = DeclareLaunchArgument(
        'enable_beat_sync',
        default_value='true',
        description='Enable phase synchronization to beats'
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

    # Music analyzer node (only if use_music is true)
    music_analyzer_node = Node(
        package='shutter_music_analyzer',
        executable='music_analyzer',
        name='music_beat_analyzer',
        output='screen',
        parameters=[{
            'music_file': LaunchConfiguration('music_file'),
            'publish_interval': 0.1,
        }],
        condition=IfCondition(LaunchConfiguration('use_music'))
    )
    
    # Oscillator control node (music-driven motion for all 4 joints)
    oscillator_node = Node(
        package='shutter_music_analyzer',
        executable='oscillator_control',
        name='oscillator_control',
        output='screen',
        parameters=[{
            'k_tempo': LaunchConfiguration('k_tempo'),
            'k_energy': LaunchConfiguration('k_energy'),
            'base_amplitude': LaunchConfiguration('base_amplitude'),
            'control_rate': LaunchConfiguration('control_rate'),
            'enable_beat_sync': LaunchConfiguration('enable_beat_sync'),
            'use_phase_offsets': 'true',  # Use phase offsets for coordinated motion
        }]
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
        use_music_arg,
        music_file_arg,
        k_tempo_arg,
        k_energy_arg,
        base_amplitude_arg,
        control_rate_arg,
        enable_beat_sync_arg,
        shutter_launch,
        face_launch,
        music_analyzer_node,
        oscillator_node,
    ])
