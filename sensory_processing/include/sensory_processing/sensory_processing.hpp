#ifndef SENSORY_PROCESSING_HPP
#define SENSORY_PROCESSING_HPP

#include <future>

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
#include "commons/Action.h"
#include "commons/commons.hpp"

namespace ml
{
    class SensoryProcessing
    {
    public:
        explicit SensoryProcessing(ros::NodeHandle &nh);
        ~SensoryProcessing() = default;

        void ProcessSensoryInputs();

    private:
        float _prev_left_wheel_joint_angle;
        float _prev_right_wheel_joint_angle;
        sensor_msgs::ImageConstPtr _prev_image;
        commons::ActionConstPtr _prev_action;
        ros::NodeHandle _nh;

        bool WheelsStateChanged();
        bool CloseToObstacle();
        bool Curiosity();
        bool CheckTopics();

        ros::Publisher _pains_pub;
        commons::PainsPtr _pains;

        constexpr static float RANGE_THRESHOLD = 0.2f;
        constexpr static float WHEEL_THRESHOLD = 0.01f; 
        constexpr static int MIN_NON_ZERO = 1000;

        struct PainIncrements
        {
            inline static float low_battery_level = 0.01;    
            inline static float mechanical_damage = 0.01;    
            inline static float curiosity = 0.01;    
        };
    };

} // namespace ml

#endif