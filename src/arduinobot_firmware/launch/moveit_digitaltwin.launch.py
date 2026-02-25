#!/usr/bin/env python3


import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def validate_port(context, *args, **kwargs):
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
        default_value="/dev/ttyACM1",
        description="Arduino serial port"
    )

    port_validator = OpaqueFunction(function=validate_port)

    hardware_bridge = Node(
        package="arduinobot_firmware",
        executable="live_hardware_controller.py",
        name="live_hardware_controller",
        output="screen",
        parameters=[{
            "port": port,
            "baudrate": 115200,
            "update_rate": 20.0
        }]
    )

    return LaunchDescription([
        port_arg,
        port_validator,
        hardware_bridge
    ])
