#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- args for joy driver ---
    joy_device_arg = DeclareLaunchArgument('joy_device', default_value='/dev/input/js0', description='Joystick device path')
    joy_autorepeat_arg = DeclareLaunchArgument('autorepeat_rate', default_value='50.0', description='Autorepeat rate (Hz)')
    joy_deadzone_arg = DeclareLaunchArgument('deadzone', default_value='0.1', description='Analog deadzone')

    # joy driver -> /man/joy
    joy_driver_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_driver',
        parameters=[{
            'dev': LaunchConfiguration('joy_device'),
            'autorepeat_rate': LaunchConfiguration('autorepeat_rate'),
            'deadzone': LaunchConfiguration('deadzone'),
            'coalesce_interval_ms': 1,
        }],
        remappings=[('/joy', '/man/joy')],
        output='screen'
    )

    # run servo_node (ของคุณ)
    servo_node = Node(
        package='servo_controller',
        executable='servo_node', 
        name='servo_node',
        output='screen'
    )

    return LaunchDescription([
        joy_device_arg, joy_autorepeat_arg, joy_deadzone_arg,
        joy_driver_node, servo_node
    ])
