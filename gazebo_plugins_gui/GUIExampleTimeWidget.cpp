#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "GUIExampleTimeWidget.hpp"

namespace gazebo
{
    // Register this plugin with the simulator
    GZ_REGISTER_GUI_PLUGIN(GUIExampleTimeWidget)

    /////////////////////////////////////////////////
    GUIExampleTimeWidget::GUIExampleTimeWidget()
        : GUIPlugin()
    {
        ROS_ERROR("GUIExampleTimeWidget");

        // Set the frame background and foreground colors
        this->setStyleSheet(
            "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

        // Create the main layout
        QHBoxLayout *mainLayout = new QHBoxLayout;

        // Create the frame to hold all the widgets
        QFrame *mainFrame = new QFrame();

        // Create the layout that sits inside the frame
        QHBoxLayout *frameLayout = new QHBoxLayout();

        QLabel *label = new QLabel(tr("Sim Time:"));

        // Create a time label
        QLabel *timeLabel = new QLabel(tr("00:00:00.00"));

        // Add the label to the frame's layout
        frameLayout->addWidget(label);
        frameLayout->addWidget(timeLabel);
        connect(this, SIGNAL(SetSimTime(QString)),
                timeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

        // Add frameLayout to the frame
        mainFrame->setLayout(frameLayout);

        // Add the frame to the main layout
        mainLayout->addWidget(mainFrame);

        // Remove margins to reduce space
        frameLayout->setContentsMargins(4, 4, 4, 4);
        mainLayout->setContentsMargins(0, 0, 0, 0);

        this->setLayout(mainLayout);

        // Position and resize this widget
        this->move(200, 10);
        this->resize(200, 40);

        // Create a node for transportation
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init("default");
        this->statsSub = this->node->Subscribe("~/world_stats",
                                               &GUIExampleTimeWidget::OnStats, this);
    }

    /////////////////////////////////////////////////
    GUIExampleTimeWidget::~GUIExampleTimeWidget()
    {
    }

    /////////////////////////////////////////////////
    void GUIExampleTimeWidget::OnStats(ConstWorldStatisticsPtr &_msg)
    {
        // ROS_ERROR_STREAM("OnStats: " << ros::Time::now());
        this->SetSimTime(QString::fromStdString(this->FormatTime(_msg->sim_time())));
    }

    /////////////////////////////////////////////////
    std::string GUIExampleTimeWidget::FormatTime(const msgs::Time &_msg) const
    {
        std::ostringstream stream;
        unsigned int day, hour, min, sec, msec;

        stream.str("");

        sec = _msg.sec();

        day = sec / 86400;
        sec -= day * 86400;

        hour = sec / 3600;
        sec -= hour * 3600;

        min = sec / 60;
        sec -= min * 60;

        msec = rint(_msg.nsec() * 1e-6);

        stream << std::setw(2) << std::setfill('0') << day << " ";
        stream << std::setw(2) << std::setfill('0') << hour << ":";
        stream << std::setw(2) << std::setfill('0') << min << ":";
        stream << std::setw(2) << std::setfill('0') << sec << ".";
        stream << std::setw(3) << std::setfill('0') << msec;

        return stream.str();
    }

} // namespace gazebo
