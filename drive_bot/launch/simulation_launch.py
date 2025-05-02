#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'drive_bot'
    pkg_share = get_package_share_directory(package_name)

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Paths to files
    world_file = os.path.join(pkg_share, 'worlds', 'simple_world.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'drive_bot.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    controller_config = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # Gazebo launch with additional parameters for gazebo_ros2_control
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file, '-s', 'libgazebo_ros2_control.so'],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': os.path.join(pkg_share, 'models')}
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'drive_bot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': open(urdf_file).read()}],
        output='screen'
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config, {'use_sim_time': use_sim_time}],
        output='screen',
        remappings=[
            ('/diff_drive_controller/cmd_vel', '/cmd_vel'),
            ('/diff_drive_controller/odom', '/odom')
        ]
    )

    # Spawn controllers with delay
    spawn_diff_drive_controller = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    spawn_joint_state_broadcaster = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
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
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        gazebo,
        spawn_entity,
        robot_state_publisher,
        controller_manager,
        spawn_diff_drive_controller,
        spawn_joint_state_broadcaster,
        rviz,
        bug_navigation_node
    ])