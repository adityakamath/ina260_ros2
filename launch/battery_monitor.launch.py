#!/usr/bin/env python3
"""Launch the INA260 battery monitor node.

Example usage:
    ros2 launch ina260_ros2 battery_monitor.launch.py
    ros2 launch ina260_ros2 battery_monitor.launch.py params_file:=/path/to/my_battery.yaml
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
            default_value=[FindPackageShare('ina260_ros2'), '/config/battery.yaml'],
            description='Path to the battery_monitor_node parameters YAML file',
        ),
    ]

    battery_monitor_node = Node(
        package='ina260_ros2',
        executable='battery_monitor_node',
        name='battery_monitor_node',
        output='log',
        respawn=True,
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription(declared_arguments + [battery_monitor_node])
