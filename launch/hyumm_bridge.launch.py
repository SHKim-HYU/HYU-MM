"""hyumm_bridge: real MRT loop (ROS2 Galactic).

  ros2 launch hyumm_bridge hyumm_bridge.launch.py send_desired:=true

Requires hyumm_mpc_node (OCS2 MPC) and hyumm_nrt's hyumm_vive_xddp_node
(vive base pose + /joint_states) running.

The bridge waits IDLE until the RT controller requests planning (START_FRONT /
START_BACK over XDDP); it then plans the front/back CSV from its row 0 and streams
pos/vel/accel. csv_front/csv_back select those trajectories.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('hyumm_bridge')
    default_config = os.path.join(pkg, 'config', 'hyumm_bridge.yaml')

    rtecat_cfg = '/home/robot/robot_ws/RTECAT_MobileManipulator/config'
    default_front = os.path.join(rtecat_cfg, 'trajectory_Christofides_LG_front_combined_10Hz.csv')
    default_back = os.path.join(rtecat_cfg, 'trajectory_Christofides_LG_back_combined_10Hz.csv')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('send_desired', default_value='false'),
        # Front/back scan trajectories the MPC plans on the RT's START_FRONT/START_BACK.
        DeclareLaunchArgument('csv_front', default_value=default_front),
        DeclareLaunchArgument('csv_back', default_value=default_back),
        # Handshake: RT->NRT command port (must match RTECAT XDDP_PORT_MPC_CMD).
        DeclareLaunchArgument('xddp_cmd_port', default_value='6'),
        Node(
            package='hyumm_bridge',
            executable='hyumm_bridge_node',
            name='hyumm_bridge_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'send_desired': LaunchConfiguration('send_desired'),
                 'csv_front': LaunchConfiguration('csv_front'),
                 'csv_back': LaunchConfiguration('csv_back'),
                 'xddp_cmd_port': ParameterValue(
                     LaunchConfiguration('xddp_cmd_port'), value_type=int)},
            ],
        ),
    ])
