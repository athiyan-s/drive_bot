#!/usr/bin/env python3
"""Launch file for simulating a differential drive robot with navigation in Gazebo."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the simulation."""
    package_name = 'drive_bot'
    pkg_share = get_package_share_directory(package_name)

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    # File paths
    world_file = os.path.join(pkg_share, 'worlds', 'simple_world.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'drive_bot.urdf')

    # Gazebo launch
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': os.path.join(pkg_share, 'models')},
        condition=IfCondition(gui)
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'drive_bot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Robot state publisher
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        output='screen'
    )

    # Bug navigation node
    bug_navigation_node = Node(
        package='drive_bot',
        executable='bug_navigation_node',
        name='bug_navigation_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run Gazebo headless'
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bug_navigation_node
    ])