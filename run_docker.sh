#!/bin/bash
source /catkin_ws/install/setup.bash
# cd /catkin_ws/src/yag-slam/ros1/
cd /home/jari/Work/yag_ws/src/yag-slam/ros1/
./slam_node_ros1 "$@"