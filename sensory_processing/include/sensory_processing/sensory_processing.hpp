#ifndef SENSORY_PROCESSING_HPP
#define SENSORY_PROCESSING_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

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
        float _prev_left_wheel_joint_angle;
        float _prev_right_wheel_joint_angle;

        sensor_msgs::ImageConstPtr _prev_image;

        bool _WheelsStateChanged();
        bool _CloseToObstacle();
        bool _Curiosity();

        ros::Publisher _pains_pub;
        commons::PainsPtr _pains;

        constexpr static float RANGE_THRESHOLD = 0.2F;

    };

} // namespace ml

#endif