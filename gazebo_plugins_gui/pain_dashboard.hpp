#ifndef PAIN_DASHBOARD_HPP
#define PAIN_DASHBOARD_HPP

// ___CPP___
#include <memory>

// ___ROS___
#include <ros/ros.h>
#include <std_msgs/Float32.h>

// ___Gazebo___
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif

// ___Other packages___
#include "commons/commons.hpp"
#include "commons/Pains.h"

namespace gazebo
{

    class GAZEBO_VISIBLE PainDashboard : public GUIPlugin
    {
        Q_OBJECT
    public:
        PainDashboard();
        virtual ~PainDashboard();
        virtual void Load(sdf::ElementPtr /*_sdf*/) override;

    signals:
        void SetBatteryLevelValue(QString _string);

    protected slots:
        void OnButton();
    
    private:
        std::shared_ptr<ros::NodeHandle> nh_;
        ros::Subscriber pain_sub_;
        void PainMsgCallback(const commons::Pains::ConstPtr &msg);

    };

} // namespace gazebo

#endif
