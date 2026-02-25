#!/bin/bash

source ~/digitaltwin_ws/install/setup.bash
PORT=${1:-/dev/ttyACM0}
ros2 launch arduinobot_firmware moveit_digitaltwin.launch.py port:=$PORT
sleep 3
echo -e "\033[92m✓ Successfully killed. Exiting...\033[0m"
