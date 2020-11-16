#ifndef TURTLEBOT3_ROBOT_HPP
#define TURTLEBOT3_ROBOT_HPP

#include "commons/commons.hpp"
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>

namespace turtlebot3
{
    class Turtlebot3
    {
    public:
        Turtlebot3();
        ~Turtlebot3();
        Turtlebot3(const Turtlebot3 &other) = delete;
        void GetPosition();
    };
} // namespace turtlebot3

#endif
