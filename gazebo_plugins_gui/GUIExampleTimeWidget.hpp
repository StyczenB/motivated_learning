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

    public:
        /// \brief Constructor
        GUIExampleTimeWidget();

        /// \brief Destructor
        virtual ~GUIExampleTimeWidget();

        /// \brief A signal used to set the sim time line edit.
    signals:
        /// \param[in] _string String representation of sim time.
        void SetSimTime(QString _string);

    protected:
        /// \brief Callback that received world statistics messages.
        /// \param[in] _msg World statistics message that is received.
        void OnStats(ConstWorldStatisticsPtr &_msg);

    private:
        /// \brief Helper function to format time string.
        /// \param[in] _msg Time message.
        /// \return Time formatted as a string.
        std::string FormatTime(const msgs::Time &_msg) const;

        /// \brief Node used to establish communication with gzserver.
        transport::NodePtr node;

        /// \brief Subscriber to world statistics messages.
        transport::SubscriberPtr statsSub;
    };
} // namespace gazebo

#endif