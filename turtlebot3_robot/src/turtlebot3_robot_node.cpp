#include "turtlebot3_robot/turtlebot3_robot.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot3_node");
    ros::NodeHandle nh;
    ROS_INFO("Started turtlebot3 node.");

    turtlebot3::Turtlebot3 robot(nh);

    ros::Rate r(100);
    while (ros::ok())
    {
        commons::Timer timer;
        robot.GetPosition();

        r.sleep();
    }
    return 0;
}
