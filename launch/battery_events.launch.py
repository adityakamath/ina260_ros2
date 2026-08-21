#!/usr/bin/env python3
"""Launch the battery threshold/status event node.

Example usage:
    ros2 launch ina260_ros2 battery_events.launch.py
    ros2 launch ina260_ros2 battery_events.launch.py params_file:=/path/to/my_battery_events.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'params_file',
            default_value=[FindPackageShare('ina260_ros2'), '/config/battery_events.yaml'],
            description='Path to the battery_events_node parameters YAML file',
        ),
    ]

    battery_events_node = Node(
        package='ina260_ros2',
        executable='battery_events_node',
        name='battery_events_node',
        output='log',
        respawn=True,
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription(declared_arguments + [battery_events_node])
