#include "pain_dashboard.hpp"

namespace gazebo
{
    GZ_REGISTER_GUI_PLUGIN(PainDashboard)

    PainDashboard::PainDashboard()
    {
        ROS_DEBUG("PainDashboard constructor called.");
    }

    void PainDashboard::Load(sdf::ElementPtr /*_sdf*/)
    {
        ROS_DEBUG("Load method called.");
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        nh_ = std::make_shared<ros::NodeHandle>();
        pain_sub_ = nh_->subscribe(commons::Topics::pains, 1, &PainDashboard::PainMsgCallback, this);

        // Set the frame background and foreground colors
        this->setStyleSheet("QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

        // Create the main layout
        QHBoxLayout *mainLayout = new QHBoxLayout;

        // Create the frame to hold all the widgets
        QFrame *mainFrame = new QFrame();

        // Create the layout that sits inside the frame
        QHBoxLayout *frameLayout = new QHBoxLayout();

        QPushButton *button = new QPushButton(tr("Press me"));
        connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));

        // QLabel *label = new QLabel(tr("Battery level:"));

        // // Create a time label
        // QLabel *timeLabel = new QLabel(tr("0.00"));

        // Add the label to the frame's layout
        frameLayout->addWidget(button);
        // frameLayout->addWidget(label);
        // frameLayout->addWidget(timeLabel);
        // connect(this, SIGNAL(SetBatteryLevelValue(QString)), timeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

        // Add frameLayout to the frame
        mainFrame->setLayout(frameLayout);

        // Add the frame to the main layout
        mainLayout->addWidget(mainFrame);

        // Remove margins to reduce space
        frameLayout->setContentsMargins(4, 4, 4, 4);
        mainLayout->setContentsMargins(0, 0, 0, 0);

        this->setLayout(mainLayout);

        // Position and resize this widget
        this->move(10, 10);
        this->resize(160, 30);
    }

    PainDashboard::~PainDashboard()
    {
    }

    void PainDashboard::OnButton()
    {
        ROS_DEBUG("Button clicked");
    }

    void PainDashboard::PainMsgCallback(const commons::Pains::ConstPtr &msg)
    {
        ROS_DEBUG("Received message with pain values.");
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << msg->curiosity;
        this->SetBatteryLevelValue(QString::fromStdString(ss.str()));
    }

} // namespace gazebo
