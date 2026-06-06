"""One-shot low-level bring-up (ROS2):
  backend  : vive_libsurvive_ros (Steam-free libsurvive driver -> /vive/* topics)
  nrt node : hyumm_vive_xddp_node (localize vive_world + mobile_base, TF, XDDP bridge)
  robot    : actual + nominal robot models (hyumm_description, frame_prefix nominal/)
  rviz     : optional (default off -- robot is headless; run rviz.launch.py elsewhere)
  teleop   : optional (default off -- raw-mode tty staircases other logs; run separately)

Toggles:
  backend:=false   assume the libsurvive backend runs elsewhere
  xddp:=false      TF/RViz only (no RT side needed)
  robot:=false     no robot models (frames-only RViz)
  rviz:=true       bundle RViz here
  teleop:=true     bundle keyboard teleop in THIS terminal
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vive_lib = get_package_share_directory('vive_libsurvive_ros')
    nrt = get_package_share_directory('hyumm_nrt')
    desc = get_package_share_directory('hyumm_description')
    vive = get_package_share_directory('hyumm_vive')

    backend = LaunchConfiguration('backend')
    xddp = LaunchConfiguration('xddp')
    robot = LaunchConfiguration('robot')
    rviz = LaunchConfiguration('rviz')
    teleop = LaunchConfiguration('teleop')

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='true'),
        DeclareLaunchArgument('xddp', default_value='true'),
        DeclareLaunchArgument('robot', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='false'),

        # Each include is wrapped in a scoped GroupAction so the launch args they
        # declare stay LOCAL. vive_compat.launch.py and vive_xddp.launch.py BOTH declare
        # a 'config' arg; without scoping, vive_compat's config=vive_compat.yaml leaks
        # into vive_xddp (includes share the parent scope by default), and vive_xddp_node
        # then tries to load vive_compat.yaml as its --params-file and aborts.

        # 1) libsurvive backend -> /vive/* topics
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(vive_lib, 'launch', 'vive_compat.launch.py')),
                condition=IfCondition(backend),
            ),
        ]),

        # 2) nrt low-level node: localize -> TF + XDDP (odom + cmd_vel + rt nominal status)
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nrt, 'launch', 'vive_xddp.launch.py')),
                launch_arguments={'send_xddp': xddp, 'recv_status': robot}.items(),
            ),
        ]),

        # 3) actual (vive) + nominal (rt, green) robot models
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(desc, 'launch', 'robot_display.launch.py')),
                condition=IfCondition(robot),
            ),
        ]),

        # 4) teleop -> /cmd_vel (forwarded to RT by the nrt node)
        Node(
            package='hyumm_teleop',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            condition=IfCondition(teleop),
        ),

        # 5) RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='vive_rviz',
            arguments=['-d', os.path.join(vive, 'rviz', 'hyumm_vive.rviz')],
            output='screen',
            condition=IfCondition(rviz),
        ),
    ])
