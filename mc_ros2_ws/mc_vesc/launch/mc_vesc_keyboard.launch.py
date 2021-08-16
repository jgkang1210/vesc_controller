#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # McVesc Node
    mc_vesc = Node(
        package='mc_vesc',
        executable='mc_vesc',
        name='mc_vesc',
        output='screen',
    )

    # teleop twist keyboard node
    teleop_twist = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        arguments=['--ros-args --remap /cmd_vel:=/cmd_vel']
    )

    return LaunchDescription([
        mc_vesc,
        teleop_twist,
    ])