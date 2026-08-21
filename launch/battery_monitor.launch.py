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
        # No respawn: a missing INA260 makes the node exit cleanly on purpose (see
        # battery_monitor_node.py's module docstring) - that's a permanent condition for
        # this launch session, not a crash respawn should try to recover from. Genuine
        # transient I2C read errors during normal operation don't exit the process at
        # all (skip-and-retry in _on_timer), so respawn has nothing left to usefully do.
        respawn=False,
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription(declared_arguments + [battery_monitor_node])
