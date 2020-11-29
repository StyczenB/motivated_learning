from rqt_gui_py.plugin import Plugin

from .agent_dashboard_widget import AgentDashboardWidget


class AgentDashboard(Plugin):
    def __init__(self, context):
        super(AgentDashboard, self).__init__(context)
        self.setObjectName('AgentDashboard')
        self._widget = AgentDashboardWidget(self)
        self._widget.start()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + f' ({context.serial_number()})')
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)
