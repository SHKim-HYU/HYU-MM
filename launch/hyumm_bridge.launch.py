"""hyumm_bridge: real MRT loop (ROS2 Galactic).

  ros2 launch hyumm_bridge hyumm_bridge.launch.py send_desired:=true

Requires hyumm_mpc_node (OCS2 MPC) and hyumm_nrt's hyumm_vive_xddp_node
(vive base pose + /joint_states) running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('hyumm_bridge')
    default_config = os.path.join(pkg, 'config', 'hyumm_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('send_desired', default_value='false'),
        Node(
            package='hyumm_bridge',
            executable='hyumm_bridge_node',
            name='hyumm_bridge_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'send_desired': LaunchConfiguration('send_desired')},
            ],
        ),
    ])
