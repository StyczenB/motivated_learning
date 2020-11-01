#include "sensory_processing/sensory_processing.hpp"

namespace ml
{
    SensoryProcessing::SensoryProcessing()
        : _prev_left_wheel_joint_angle(0), _prev_right_wheel_joint_angle(0)
    {
        ros::NodeHandle nh("~");
        _pains_pub = nh.advertise<commons::Pains>(commons::Topics::pains, 1, true);
        _pains = boost::make_shared<commons::Pains>();
        _pains->header.frame_id = "base_footprint";
        _prev_image = boost::make_shared<sensor_msgs::Image>();
        _prev_action = boost::make_shared<commons::Action>();
    }

    SensoryProcessing::~SensoryProcessing()
    {
    }

    void SensoryProcessing::ProcessSensoryInputs()
    {
        std::future<bool> wheels_state = std::async(std::launch::async, &SensoryProcessing::_WheelsStateChanged, this);
        std::future<bool> close_to_obstacle = std::async(std::launch::async, &SensoryProcessing::_CloseToObstacle, this);
        std::future<bool> curiosity = std::async(std::launch::async, &SensoryProcessing::_Curiosity, this);

        // Low battery level pain
        if (wheels_state.get())
        {
            ROS_DEBUG("Wheels state changed. Pain of low battery level increases.");
            _pains->low_battery_level += PainIncrements::low_battery_level;
        }

        // Mechanical damage pain
        if (close_to_obstacle.get())
        {
            ROS_DEBUG("Robot is too close to some obstacle e.g. wall. Pain of mechanical damage increases.");
            _pains->mechanical_damage += PainIncrements::low_battery_level;
        }

        // Curiosity
        if (curiosity.get())
        {
            ROS_DEBUG("Something with curiosity happened. Do something boiii.");
            _pains->curiosity += PainIncrements::curiosity;
        }

        _pains->header.seq++;
        _pains->header.stamp = ros::Time::now();

        _pains_pub.publish(_pains);
    }

    bool SensoryProcessing::_WheelsStateChanged()
    {
        commons::Timer timer("SensoryProcessing::_WheelsStateChanged");

        sensor_msgs::JointStateConstPtr wheels_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

        auto left_wheel_joint_name_it = std::find(wheels_state->name.begin(), wheels_state->name.end(), "wheel_left_joint");
        if (left_wheel_joint_name_it == wheels_state->name.end())
        {
            ROS_FATAL("Missing left wheel joint state. Exiting...");
            std::exit(EXIT_FAILURE);
        }

        auto right_wheel_joint_name_it = std::find(wheels_state->name.begin(), wheels_state->name.end(), "wheel_right_joint");
        if (right_wheel_joint_name_it == wheels_state->name.end())
        {
            ROS_FATAL("Missing right wheel joint state. Exiting...");
            std::exit(EXIT_FAILURE);
        }

        float left_wheel_joint_angle = wheels_state->position[left_wheel_joint_name_it - wheels_state->name.begin()];
        float right_wheel_joint_angle = wheels_state->position[right_wheel_joint_name_it - wheels_state->name.begin()];
        // ROS_DEBUG_STREAM("\nl: " << left_wheel_joint_angle << "\nr: " << right_wheel_joint_angle);

        bool left_changed = std::fabs(left_wheel_joint_angle - _prev_left_wheel_joint_angle) > 0.01;
        bool right_changed = std::fabs(right_wheel_joint_angle - _prev_right_wheel_joint_angle) > 0.01;

        _prev_left_wheel_joint_angle = left_wheel_joint_angle;
        _prev_right_wheel_joint_angle = right_wheel_joint_angle;

        return left_changed || right_changed;
    }

    bool SensoryProcessing::_CloseToObstacle()
    {
        commons::Timer timer("SensoryProcessing::_CloseToObstacle");

        sensor_msgs::LaserScanConstPtr scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>(commons::Topics::laser_scan, ros::Duration(0.5));
        if (scan == nullptr)
        {
            ROS_FATAL("There is no laser scan data. Exiting...");
            std::exit(EXIT_FAILURE);
        }

        bool too_close = false;
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            if (scan->ranges[i] != std::numeric_limits<float>::infinity())
            {
                if (scan->ranges[i] < RANGE_THRESHOLD)
                    too_close = true;
                break;
            }
        }
        return too_close;
    }

    bool SensoryProcessing::_Curiosity()
    {
        commons::Timer timer("SensoryProcessing::_Curiosity");

        sensor_msgs::ImageConstPtr rgb = ros::topic::waitForMessage<sensor_msgs::Image>(commons::Topics::rgb, ros::Duration(0.5));
        if (rgb == nullptr)
        {
            ROS_FATAL("There is no video feed. Exiting...");
            std::exit(EXIT_FAILURE);
        }

        auto current_cv_ptr = cv_bridge::toCvShare(rgb);
        cv::Mat current_gray;
        cv::cvtColor(current_cv_ptr->image, current_gray, cv::COLOR_RGB2GRAY);

        int ones = 0;
        if (!_prev_image->data.empty())
        {
            auto prev_cv_ptr = cv_bridge::toCvShare(_prev_image);
            cv::Mat prev_gray;
            cv::cvtColor(prev_cv_ptr->image, prev_gray, cv::COLOR_RGB2GRAY);

            cv::Mat diff;
            cv::absdiff(current_gray, prev_gray, diff);

            cv::Mat mask;
            cv::threshold(diff, mask, 1, 255, cv::THRESH_BINARY);

            ones = cv::countNonZero(mask);
            // cv::imshow("mask", mask);
            // cv::waitKey(1);
        }
        _prev_image = rgb;
        bool scene_changed = ones < MIN_NON_ZERO;

        // Check action
        bool action_changed = false;
        // commons::ActionConstPtr action = ros::topic::waitForMessage<commons::Action>(commons::Topics::action, ros::Duration(0.5));
        // if (action == nullptr)
        // {
        //     ROS_FATAL("No action.");
        // }

        return scene_changed || action_changed;
    }

}; // namespace ml
