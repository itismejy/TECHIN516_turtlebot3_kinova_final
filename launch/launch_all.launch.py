#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch the kortex bringup launch file
    kortex_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('kortex_bringup'),
                'launch',
                'gen3_lite.launch.py'
            )
        ),
        launch_arguments={
            'robot_ip': '10.18.2.240',
            'launch_rviz': 'false'
        }.items()
    )

    # Launch the kinova MoveIt launch file
    kinova_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('kinova_gen3_lite_moveit_config'),
                'launch',
                'robot.launch.py'
            )
        ),
        launch_arguments={
            'robot_ip': '10.18.2.240'
        }.items()
    )

    # Delay execution of the pick_place_move node to ensure previous nodes are up
    pick_place_move = TimerAction(
        period=5.0,  # adjust delay as needed
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', '516_final', 'pick_place_move'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        kortex_launch,
        kinova_launch,
        pick_place_move
    ])

