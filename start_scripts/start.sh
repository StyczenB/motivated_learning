#!/bin/bash

source ../../devel/setup.bash
export GAZEBO_MODEL_PATH=../turtlebot3_gazebo/models/:$GAZEBO_MODEL_PATH
catkin build

if [ $? -eq 0 ]; then
    ./kill.sh
    roslaunch simulation.launch &
    sleep 3
    roslaunch algorithms.launch
fi
