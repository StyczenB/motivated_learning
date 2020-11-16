#include "turtlebot3_robot/turtlebot3_robot.hpp"

namespace turtlebot3
{
    Turtlebot3::Turtlebot3()
    {
    }

    Turtlebot3::~Turtlebot3()
    {
    }

    void Turtlebot3::GetPosition()
    {
        ROS_DEBUG("Getting robots position in the world.");

        gazebo_msgs::ModelStatesPtr model_states = ros::topic::waitForMessage<gazebo_msgs::ModelStates>(commons::Topics::model_states);
        if (model_states == nullptr)
        {
            std::exit(EXIT_FAILURE);
        }
    }

} // namespace turtlebot3