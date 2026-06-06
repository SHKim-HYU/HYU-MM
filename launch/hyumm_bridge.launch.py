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

    # Front/back scan trajectories = the re-planned CSVs from hyumm_scan_moveit_config
    # (data/front, data/back), Christofides_LG variant. Same source the RViz playback uses,
    # so the bridge and the visualization stay in sync. Override csv_front/csv_back to switch.
    moveit_data = os.path.join(
        get_package_share_directory('hyumm_scan_moveit_config'), 'data')
    default_front = os.path.join(
        moveit_data, 'front', 'trajectory_Christofides_LG_front_combined_10Hz.csv')
    default_back = os.path.join(
        moveit_data, 'back', 'trajectory_Christofides_LG_back_combined_10Hz.csv')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('send_desired', default_value='false'),
        # Front/back scan trajectories the MPC plans on the RT's START_FRONT/START_BACK.
        DeclareLaunchArgument('csv_front', default_value=default_front),
        DeclareLaunchArgument('csv_back', default_value=default_back),
        # Handshake: RT->NRT command port (must match RTECAT XDDP_PORT_MPC_CMD).
        DeclareLaunchArgument('xddp_cmd_port', default_value='6'),
        # ROS2 verification (no Xenomai/XDDP): mirror XDDP to ROS topics + loopback.
        # Drive ~/mpc_cmd (1 START_FRONT, 2 START_BACK), echo ~/desired & ~/mpc_status.
        # Override the slip with -p loopback_base_error:=[0.1,0.0,0.0].
        DeclareLaunchArgument('ros_debug', default_value='false'),
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
                     LaunchConfiguration('xddp_cmd_port'), value_type=int),
                 'ros_debug': ParameterValue(
                     LaunchConfiguration('ros_debug'), value_type=bool)},
            ],
        ),
    ])
