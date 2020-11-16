#!/bin/bash
source /home/bartek/study/repo/devel/setup.bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
KILL_DIR="${DIR}/kill.sh"
/bin/bash $KILL_DIR
catkin build
roslaunch turtlebot3_gazebo start.launch
