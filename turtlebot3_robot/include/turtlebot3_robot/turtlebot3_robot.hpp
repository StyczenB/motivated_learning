#ifndef TURTLEBOT3_ROBOT_HPP
#define TURTLEBOT3_ROBOT_HPP

#include "commons/commons.hpp"
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>

namespace turtlebot3
{
    class Turtlebot3
    {
    public:
        Turtlebot3(ros::NodeHandle &nh);
        ~Turtlebot3();
        Turtlebot3(const Turtlebot3 &other) = delete;
        void GetPosition();
    private:
        ros::NodeHandle _nh;
    };
} // namespace turtlebot3

#endif
