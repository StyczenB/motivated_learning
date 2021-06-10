#!/bin/bash

killall -9 gzclient
killall -9 gzserver
# killall -9 goal_creation_system_node
# killall -9 sensory_processing_node
killall -9 robot_state_publisher
killall -9 move_base
killall -9 rviz

exit 0
