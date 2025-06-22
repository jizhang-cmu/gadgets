#!/bin/bash

export ROS_DOMAIN_ID=1

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./install/setup.bash
ros2 run rviz2 rviz2 -d rviz/data_view_with_route_planner.rviz
