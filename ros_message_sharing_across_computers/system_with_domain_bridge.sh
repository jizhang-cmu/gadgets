#!/bin/bash

cd ~/autonomy_stack_mecanum_wheel_platform
source ./install/setup.bash
ros2 launch vehicle_simulator system_real_robot.launch &
sleep 1
ros2 launch domain_bridge domain_bridge.launch
