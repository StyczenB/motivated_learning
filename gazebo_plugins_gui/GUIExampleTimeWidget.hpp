#ifndef _GUI_EXAMPLE_TIME_WIDGET_HH_
#define _GUI_EXAMPLE_TIME_WIDGET_HH_

#include <string>

#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
// moc parsing error of tbb headers
#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif

namespace gazebo
{
    class GAZEBO_VISIBLE GUIExampleTimeWidget : public GUIPlugin
    {
        Q_OBJECT

        /// \brief Constructor
    public:
        GUIExampleTimeWidget();

        /// \brief Destructor
    public:
        virtual ~GUIExampleTimeWidget();

        /// \brief A signal used to set the sim time line edit.
        /// \param[in] _string String representation of sim time.
    signals:
        void SetSimTime(QString _string);

        /// \brief Callback that received world statistics messages.
        /// \param[in] _msg World statistics message that is received.
    protected:
        void OnStats(ConstWorldStatisticsPtr &_msg);

        /// \brief Helper function to format time string.
        /// \param[in] _msg Time message.
        /// \return Time formatted as a string.
    private:
        std::string FormatTime(const msgs::Time &_msg) const;

        /// \brief Node used to establish communication with gzserver.
    private:
        transport::NodePtr node;

        /// \brief Subscriber to world statistics messages.
    private:
        transport::SubscriberPtr statsSub;
    };
} // namespace gazebo

#endif