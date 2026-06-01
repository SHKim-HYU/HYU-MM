"""Merge the HYU-MM dual rplidar scans into /merged_cloud + /scan_multi (ROS2).

  ros2 launch ira_laser_tools laserscan_multi_merger.launch.py

Subscribes /rplidar0/scan + /rplidar1/scan (the dual A2M12 setup) and merges
them in the base_link frame. Start the lidars first, e.g.:
  ros2 launch rplidar_ros multi_rplidar_a2m12_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            output='screen',
            parameters=[{
                'destination_frame': 'base_link',
                'cloud_destination_topic': '/merged_cloud',
                'scan_destination_topic': '/scan_multi',
                'laserscan_topics': '/rplidar0/scan /rplidar1/scan',
                'angle_min': -3.141592,
                'angle_max': 3.141592,
                'angle_increment': 0.0058,
                'scan_time': 0.0333333,
                'range_min': 0.30,
                'range_max': 50.0,
            }],
        ),
    ])
