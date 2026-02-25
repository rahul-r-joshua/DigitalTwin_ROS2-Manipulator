#!/usr/bin/env python3

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def validate_port(context, *args, **kwargs):
    """Validate serial port exists before launching"""
    port_value = LaunchConfiguration('port').perform(context)
    
    if not os.path.exists(port_value):
        print(f"\n❌ ERROR: Serial port '{port_value}' does not exist!")
        print(f"Please check:")
        print(f"  1. Arduino is connected via USB")
        print(f"  2. Correct port (try: ls /dev/ttyACM* or ls /dev/ttyUSB*)")
        print(f"  3. User has permissions: sudo usermod -a -G dialout $USER\n")
        sys.exit(1)
    
    return []

def generate_launch_description():
    port = LaunchConfiguration("port")
    
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Arduino serial port"
    )
    
    port_validator = OpaqueFunction(function=validate_port)

    description_pkg_share = get_package_share_directory('arduinobot_description')
    rviz_config_file = os.path.join(description_pkg_share, 'rviz', 'display.rviz')
    urdf_file = os.path.join(description_pkg_share, 'urdf', 'arduinobot.urdf.xacro')

    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    joint_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    gui_hardware = Node(
        package="arduinobot_firmware",
        executable="gui_hardware_controller",
        name="gui_hardware_controller",
        output="screen",
        parameters=[{
            "port": port,
            "baudrate": 115200,
            "update_rate": 20.0
        }]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_file]
    )


    return LaunchDescription([
        port_arg,
        port_validator,
        robot_state_publisher,
        joint_gui,
        gui_hardware,
        rviz
    ])