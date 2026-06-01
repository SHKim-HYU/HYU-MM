"""Standalone RViz view of vive_world + actual(vive) / nominal(green) robots (ROS2).

  ros2 launch hyumm_bringup rviz.launch.py
  ros2 launch hyumm_bringup rviz.launch.py robot:=true   # also start robot models here

robot:=true also starts the actual+nominal robot_state_publishers here (use only
when nothing else publishes them). Needs hyumm_description on this machine so the
package:// meshes resolve.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vive_share = get_package_share_directory('hyumm_vive')
    desc_share = get_package_share_directory('hyumm_description')
    rviz_config = os.path.join(vive_share, 'rviz', 'hyumm_vive.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_share, 'launch', 'robot_display.launch.py')),
            condition=IfCondition(LaunchConfiguration('robot')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='vive_rviz',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
