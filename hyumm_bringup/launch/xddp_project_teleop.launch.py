"""Run the RT-odom -> /odom XDDP reader node (ROS2).

  ros2 launch hyumm_bringup xddp_project_teleop.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hyumm_nrt',
            executable='hyumm_xddp_node',
            name='hyumm_xddp_node',
            output='screen',
        ),
    ])
