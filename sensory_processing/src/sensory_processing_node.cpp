#include <iostream>
#include <ros/ros.h>
#include "sensory_processing/sensory_processing.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "system_processing_node");
    ros::NodeHandle nh;
    ROS_INFO("Started sensory processing node.");
    
    ml::SensoryProcessing sensory_processing(nh);

    ros::Rate r(100);
    while (ros::ok())
    {
        commons::Timer timer;
        sensory_processing.ProcessSensoryInputs();

        r.sleep();
    }

    return 0;
}
