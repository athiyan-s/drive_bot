#!/usr/bin/env python3

"""Launch file for Gazebo simulation and teleoperation of the drive_bot robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('drive_bot')

    # Launch arguments
    world_file = LaunchConfiguration('world_file')
    urdf_file = LaunchConfiguration('urdf_file')
    controllers_file = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            default_value='src/drive_bot/worlds/simple_world.world',
            description='Path to the Gazebo world file'
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=PathJoinSubstitution([pkg_share, 'urdf', 'drive_bot.urdf.xacro']),
            description='Path to the robot URDF file'
        ),

        # Gazebo server
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen',
            name='gazebo_server'
        ),

        # Gazebo client
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            name='gazebo_client'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'drive_bot', '-topic', '/robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen',
            name='spawn_entity'
        ),

        # Load controllers (diff_cont and joint_broad)
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_cont'],
                    output='screen',
                    name='diff_cont_spawner'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_broad'],
                    output='screen',
                    name='joint_broad_spawner'
                ),
            ]
        ),
    ])