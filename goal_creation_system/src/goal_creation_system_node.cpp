#include <iostream>
#include <ros/ros.h>
#include "commons/commons.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_creation_system_node");
    ros::NodeHandle nh;

    ROS_INFO("Started goal creation system (GCS) node");

    ros::Rate r(100);
    while (ros::ok())
    {

        r.sleep();
    }

    return 0;
}
