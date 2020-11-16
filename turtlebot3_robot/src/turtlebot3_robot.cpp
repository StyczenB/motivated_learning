#include "turtlebot3_robot/turtlebot3_robot.hpp"

namespace turtlebot3
{
    Turtlebot3::Turtlebot3(ros::NodeHandle &nh)
        : _nh(nh)
    {
    }

    Turtlebot3::~Turtlebot3()
    {
    }

    void Turtlebot3::GetPosition()
    {
        ROS_DEBUG("Getting robots position in the world.");

        // ros::ServiceClient model_state_client = _nh.serviceClient<gazebo_msgs::GetModelState>(commons::GazeboServices::model_state);
        // gazebo_msgs::GetModelState srv;
        // srv.request.model_name = commons::Models::turtlebot3;
        // if (!model_state_client.call(srv))
        // {  
        //     ROS_ERROR("Could not call gazebo get model state.");
        // }

        // float x_pos = srv.response.pose.position.x;
        // float y_pos = srv.response.pose.position.y;

        // float yaw = srv.response.pose.orientation.
    }

} // namespace turtlebot3