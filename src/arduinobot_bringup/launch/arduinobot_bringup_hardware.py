#!/usr/bin/env python3
"""
ArduinoBot Bringup - Complete System Launch
Launches: Gazebo → Controller → MoveIt
"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    description_pkg_share = get_package_share_directory('arduinobot_description')
    controller_pkg_share = get_package_share_directory('arduinobot_controller')
    moveit_pkg_share = get_package_share_directory('arduinobot_moveit')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(description_pkg_share, 'launch', 'gazebo.launch.py')
        ])
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(controller_pkg_share, 'launch', 'controller.launch.py')
        ])
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(moveit_pkg_share, 'launch', 'moveit.launch.py')
        ])
    )

    delay_controller = TimerAction(
        period=3.0,
        actions=[controller_launch]
    )

    delay_moveit = TimerAction(
        period=6.0,
        actions=[moveit_launch]
    )

    return LaunchDescription([
        gazebo_launch,
        delay_controller,
        delay_moveit
    ])

 
