cmake_minimum_required(VERSION 3.0.2)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  commons
  robot_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot3_robot
 CATKIN_DEPENDS robot_msgs
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/movement_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
