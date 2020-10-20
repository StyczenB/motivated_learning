cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gazebo_plugins_gui)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dupsko
 CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(SYSTEM 
  # include
  ${GAZEBO_INCLUDE_DIRS} 
  ${Qt5Core_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(gui_example SHARED 
  ${headers_MOC}
  GUIExampleTimeWidget.cpp
  # src/GUIExampleTimeWidget.cpp
)
target_link_libraries(gui_example
  ${catkin_LIBRARIES}    
  ${GAZEBO_LIBRARIES}
  ${OGRE_LIBRARIES}
  ${OGRE-Terrain_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

add_library(gui_spawn_widget SHARED 
  ${headers_MOC}
  ExampleSpawnWidget.cc
)
target_link_libraries(gui_spawn_widget 
  ${catkin_LIBRARIES}    
  ${GAZEBO_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)
