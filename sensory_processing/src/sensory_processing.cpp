#include "sensory_processing/sensory_processing.hpp"

namespace ml
{
    SensoryProcessing::SensoryProcessing()
        : _prev_left_wheel_joint_angle(0), _prev_right_wheel_joint_angle(0)
    {
        ros::NodeHandle nh("~");
        _pains_pub = nh.advertise<commons::Pains>(commons::Topics::pains, 1, true);
        _pains = boost::make_shared<commons::Pains>();
        _prev_image = boost::make_shared<sensor_msgs::Image>();
    }

    SensoryProcessing::~SensoryProcessing()
    {
    }

    void SensoryProcessing::ProcessSensoryInputs()
    {
        // Low battery level pain
        auto start = commons::now();
        bool wheels_state_changed = _WheelsStateChanged();
        ROS_DEBUG_STREAM("_WheelsStateChanged: " << commons::elapsed(commons::now(), start) << " ms");

        if (wheels_state_changed)
        {
            ROS_DEBUG("Wheels state changed. Pain of low battery level increases.");
            _pains->low_battery_level += 0.01;
        }

        // Mechanical damage pain
        start = commons::now();
        bool close_to_obstacle = _CloseToObstacle();
        ROS_DEBUG_STREAM("_CloseToObstacle: " << commons::elapsed(commons::now(), start) << " ms");

        if (close_to_obstacle)
        {
            ROS_DEBUG("Robot is too close to some obstacle e.g. wall. Pain of mechanical damage increases.");
            _pains->mechanical_damage += 0.01;
        }

        // Curiosity
        start = commons::now();
        bool curiosity = _Curiosity();
        ROS_DEBUG_STREAM("_Curiosity: " << commons::elapsed(commons::now(), start) << " ms");

        if (curiosity)
        {
            ROS_DEBUG("Something with curiosity happened. Do something boiii.");
            _pains->curiosity += 0.01;
        }

        _pains->header.seq++;
        _pains->header.stamp = ros::Time::now();

        _pains_pub.publish(_pains);
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

        bool left_changed = std::fabs(left_wheel_joint_angle - _prev_left_wheel_joint_angle) > 0.01;
        bool right_changed = std::fabs(right_wheel_joint_angle - _prev_right_wheel_joint_angle) > 0.01;

        _prev_left_wheel_joint_angle = left_wheel_joint_angle;
        _prev_right_wheel_joint_angle = right_wheel_joint_angle;

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

    bool SensoryProcessing::_Curiosity()
    {
        sensor_msgs::ImageConstPtr rgb = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_raw");

        bool img_diff = (rgb->data != _prev_image->data);

        // auto cv_ptr = cv_bridge::toCvShare(rgb);
        // cv::Mat t;
        // cv::cvtColor(cv_ptr->image, t, cv::COLOR_BGR2GRAY);

        // cv::imshow("rgb", cv_ptr->image);
        // cv::waitKey(1);

        _prev_image = rgb;
        return img_diff;
    }

}; // namespace ml
