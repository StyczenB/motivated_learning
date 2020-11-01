#include <iostream>
#include <ros/ros.h>
#include "sensory_processing/sensory_processing.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;

    ml::SensoryProcessing sensory_processing;

    ros::Rate r(100);
    while (ros::ok())
    {
        commons::Timer timer;
        sensory_processing.ProcessSensoryInputs();

        r.sleep();
    }

    return 0;
}
