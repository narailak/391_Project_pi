#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Launch file for joystick-controlled robot drive
- Launches joystick_node.py to read joystick input
- Launches drive_node.py to control robot motors
- LY axis controls forward/backward movement
- RX axis controls turning
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Declare launch arguments
    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='1.0',
        description='Maximum linear speed (m/s)'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='3.0',
        description='Maximum angular speed (rad/s)'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug output'
    )

    # Joy driver node (built-in ROS2 joy package)
    joy_driver_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_driver',
        parameters=[{
            'dev': LaunchConfiguration('joy_device'),
            'autorepeat_rate': 50.0,  # 50 Hz
            'deadzone': 0.1,
            'coalesce_interval_ms': 1
        }],
        remappings=[
            ('/joy', '/man/joy')  # Remap to match joystick_node expectation
        ]
    )

    # Joystick mapper node
    joystick_node = Node(
        package='joystick_controller',  # Your package name
        executable='joystick_node',
        name='joystick_mapper',
        parameters=[{
            'joy_topic': '/man/joy',
            'cmd_vel_topic': '/man/cmd_move',  # Output to drive_node input
            # 'max_linear': LaunchConfiguration('max_linear_speed'),
            # 'max_angular': LaunchConfiguration('max_angular_speed'),
            # 'debug_pub': LaunchConfiguration('debug_mode')
        }],
        output='screen'
    )

    # Drive controller node
    drive_node = Node(
        package='drive_controller',  # Your package name
        executable='drive_node',
        name='drive_controller',
        parameters=[{
            # Add any parameters your drive_node might need
            'wheel_base': 0.2,
            'wheel_radius': 0.060,
            'max_speed': 1023,
            'max_rpm': 150,
            'max_linear_speed': 3.0
        }],
        output='screen'
    )

    # Return launch description
    return LaunchDescription([
        # Launch arguments
        joy_device_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        debug_mode_arg,
        
        # Nodes
        joy_driver_node,
        joystick_node,
        drive_node
    ])