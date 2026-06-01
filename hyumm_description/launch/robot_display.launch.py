"""Actual + nominal robot models in RViz (ROS2).

Joint angles come from the nrt node (hyumm_vive_xddp) over XDDP from the RT
controller:
  /joint_states          = RT act.q arm -> rsp_actual
  /nominal/joint_states  = RT nom.q arm -> rsp_nominal
Base TF: vive_world->base_link (actual, from vive) and
vive_world->nominal/base_link (nominal, from RT) are broadcast by the nrt node.

Unlike Noetic, ROS2 robot_state_publisher honours the ``frame_prefix``
parameter, so the nominal ghost reuses the SAME URDF with frame_prefix
``nominal/`` instead of a separately-prefixed URDF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg = get_package_share_directory('hyumm_description')
    default_urdf = os.path.join(pkg, 'urdf', 'hyumm_scan.urdf')

    urdf_arg = DeclareLaunchArgument('urdf', default_value=default_urdf)
    nominal_arg = DeclareLaunchArgument('nominal', default_value='true')

    # robot_description is read at launch time from the resolved urdf path.
    with open(default_urdf, 'r') as f:
        robot_desc = f.read()

    rsp_actual = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rsp_actual',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    nominal_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('nominal')),
        actions=[
            PushRosNamespace('nominal'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='rsp_nominal',
                output='screen',
                parameters=[{
                    'robot_description': robot_desc,
                    'frame_prefix': 'nominal/',
                }],
                remappings=[('joint_states', '/nominal/joint_states')],
            ),
        ],
    )

    return LaunchDescription([urdf_arg, nominal_arg, rsp_actual, nominal_group])
