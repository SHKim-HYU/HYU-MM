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
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('hyumm_bridge')
    default_config = os.path.join(pkg, 'config', 'hyumm_bridge.yaml')

    # Front/back scan CSVs live in hyumm_scan_moveit_config/share/data/{front,back} as
    # trajectory_<variant>_<front|back>_combined_10Hz.csv. Select with variant:=<algo>
    # (TR_TSP | TR_TSP_LG | Christofides_LG | TwoOpt_LG); defaults to TR_TSP to match the
    # RT controller's WITH_TR_TSP build. csv_front/csv_back still override the full path.
    data = os.path.join(
        get_package_share_directory('hyumm_scan_moveit_config'), 'data')
    variant = LaunchConfiguration('variant')
    front_default = [TextSubstitution(text=os.path.join(data, 'front', 'trajectory_')),
                     variant, TextSubstitution(text='_front_combined_10Hz.csv')]
    back_default = [TextSubstitution(text=os.path.join(data, 'back', 'trajectory_')),
                    variant, TextSubstitution(text='_back_combined_10Hz.csv')]

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('send_desired', default_value='false'),
        # Scan trajectory algorithm variant = the middle token of the CSV filename.
        DeclareLaunchArgument('variant', default_value='TR_TSP'),
        # Front/back scan trajectories the MPC plans on the RT's START_FRONT/START_BACK;
        # default to the <variant> CSVs, override either for a fully custom path.
        DeclareLaunchArgument('csv_front', default_value=front_default),
        DeclareLaunchArgument('csv_back', default_value=back_default),
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
