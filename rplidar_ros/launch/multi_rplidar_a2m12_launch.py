#!/usr/bin/env python3
"""HYU-MM dual RPLIDAR A2M12 (front + rear) for ROS2.

Front lidar: /dev/rplidar_front -> frame laser0, topic /rplidar0/scan
Rear  lidar: /dev/rplidar_rear  -> frame laser1, topic /rplidar1/scan

(Recreated from the Noetic multi_rplidar_a2m12.launch on top of the official
Slamtec ROS2 driver. udev rules should map the two devices to the stable
/dev/rplidar_front and /dev/rplidar_rear symlinks.)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rplidar(ns, default_port, frame):
    port = LaunchConfiguration(ns + '_serial_port')
    return [
        DeclareLaunchArgument(ns + '_serial_port', default_value=default_port),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            namespace=ns,
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': port,
                'serial_baudrate': 256000,
                'frame_id': frame,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }],
            output='screen'),
    ]


def generate_launch_description():
    actions = []
    actions += _rplidar('rplidar0', '/dev/rplidar_front', 'laser0')
    actions += _rplidar('rplidar1', '/dev/rplidar_rear', 'laser1')
    return LaunchDescription(actions)
