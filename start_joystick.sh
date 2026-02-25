#!/bin/bash

source ~/digitaltwin_ws/install/setup.bash
PORT=${1:-/dev/ttyACM0}
ros2 control switch_controllers --deactivate joint_state_broadcaster --strict
ros2 launch arduinobot_firmware joystick_control.launch.py port:=$PORT
sleep 3
ros2 control switch_controllers --activate joint_state_broadcaster --strict
echo -e "\033[92m✓ joint_state_broadcaster reactivated. Exiting...\033[0m"
