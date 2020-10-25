#ifndef SENSORY_PROCESSING_HPP
#define SENSORY_PROCESSING_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "commons/Pains.h"
#include "commons/commons.hpp"

namespace ml
{

    class SensoryProcessing
    {
    public:
        SensoryProcessing();
        ~SensoryProcessing();

        void ProcessSensoryInputs();

    private:
        float prev_left_wheel_joint_angle_;
        float prev_right_wheel_joint_angle_;

        bool _WheelsStateChanged();
        bool _CloseToObstacle();

        constexpr static float RANGE_THRESHOLD = 0.2F;

    };

} // namespace ml

#endif