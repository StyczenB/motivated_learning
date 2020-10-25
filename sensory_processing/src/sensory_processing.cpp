#include "sensory_processing/sensory_processing.hpp"

namespace ml
{
    SensoryProcessing::SensoryProcessing()
        : prev_left_wheel_joint_angle_(0), prev_right_wheel_joint_angle_(0)
    {
    }

    SensoryProcessing::~SensoryProcessing()
    {
    }

    void SensoryProcessing::ProcessSensoryInputs()
    {
        if (_WheelsStateChanged())
        {
            ROS_DEBUG("Wheels state changed. Pain of low battery level increases.");
        }

        if (_CloseToObstacle())
        {
            ROS_DEBUG("Robot is too close to some obstacle e.g. wall. Pain of mechanical damage increases.");
        }
    }

    bool SensoryProcessing::_WheelsStateChanged()
    {
        sensor_msgs::JointState wheels_state = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states"));

        auto left_wheel_joint_name_it = std::find(wheels_state.name.begin(), wheels_state.name.end(), "wheel_left_joint");
        if (left_wheel_joint_name_it == wheels_state.name.end())
        {
            ROS_ERROR("Missing left wheel joint state. Exiting...");
            std::exit(EXIT_FAILURE);
        }

        auto right_wheel_joint_name_it = std::find(wheels_state.name.begin(), wheels_state.name.end(), "wheel_right_joint");
        if (right_wheel_joint_name_it == wheels_state.name.end())
        {
            ROS_ERROR("Missing right wheel joint state. Exiting...");
            std::exit(EXIT_FAILURE);
        }

        float left_wheel_joint_angle = wheels_state.position[left_wheel_joint_name_it - wheels_state.name.begin()];
        float right_wheel_joint_angle = wheels_state.position[right_wheel_joint_name_it - wheels_state.name.begin()];
        // ROS_DEBUG_STREAM("\nl: " << left_wheel_joint_angle << "\nr: " << right_wheel_joint_angle);

        bool left_changed = std::fabs(left_wheel_joint_angle - prev_left_wheel_joint_angle_) > 0.01;
        bool right_changed = std::fabs(right_wheel_joint_angle - prev_right_wheel_joint_angle_) > 0.01;

        prev_left_wheel_joint_angle_ = left_wheel_joint_angle;
        prev_right_wheel_joint_angle_ = right_wheel_joint_angle;

        return left_changed || right_changed;
    }

    bool SensoryProcessing::_CloseToObstacle()
    {
        sensor_msgs::LaserScan scan = *(ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan"));
        bool too_close = false;
        for (size_t i = 0; i < scan.ranges.size(); ++i)
        {
            if (scan.ranges[i] != std::numeric_limits<float>::infinity())
            {
                if (scan.ranges[i] < RANGE_THRESHOLD)
                    too_close = true;
                break;
            }
        }
        return too_close;
    }

}; // namespace ml
