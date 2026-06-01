"""nrt low-level node (ROS2): read libsurvive topics -> localize (vive_world
anchor + mobile_base) -> broadcast TF + XDDP bridge (odom + forwarded cmd_vel).

  ros2 launch hyumm_nrt vive_xddp.launch.py send_xddp:=true recv_status:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('hyumm_nrt')
    default_config = os.path.join(pkg, 'config', 'vive_xddp.yaml')

    config = LaunchConfiguration('config')
    send_xddp = LaunchConfiguration('send_xddp')
    recv_status = LaunchConfiguration('recv_status')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        # true: stream odom + cmd_vel to the RT side
        DeclareLaunchArgument('send_xddp', default_value='false'),
        # true: receive rt NOMINAL base status (green robot)
        DeclareLaunchArgument('recv_status', default_value='false'),
        Node(
            package='hyumm_nrt',
            executable='hyumm_vive_xddp_node',
            name='hyumm_vive_xddp',
            output='screen',
            parameters=[
                config,
                {'send_xddp': send_xddp, 'recv_status': recv_status},
            ],
        ),
    ])
