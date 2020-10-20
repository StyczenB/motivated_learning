#ifndef _GUI_EXAMPLE_SPAWN_WIDGET_HH_
#define _GUI_EXAMPLE_SPAWN_WIDGET_HH_

#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
// moc parsing error of tbb headers
#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif

namespace gazebo
{
    class GAZEBO_VISIBLE GUIExampleSpawnWidget : public GUIPlugin
    {
        Q_OBJECT

    public:
        /// \brief Constructor
        /// \param[in] _parent Parent widget
        GUIExampleSpawnWidget();

        /// \brief Destructor
        virtual ~GUIExampleSpawnWidget();

    protected slots:
        /// \brief Callback trigged when the button is pressed.
        void OnButton();

    private:
        /// \brief Counter used to create unique model names
        unsigned int counter;

        /// \brief Node used to establish communication with gzserver.
        transport::NodePtr node;

        /// \brief Publisher of factory messages.
        transport::PublisherPtr factoryPub;
    };
} // namespace gazebo
#endif