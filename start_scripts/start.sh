#!/bin/bash

source ../../devel/setup.bash
catkin build

if [ $? -eq 0 ]; then
    ./kill.sh
    roslaunch start.launch
fi
