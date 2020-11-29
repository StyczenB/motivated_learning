from __future__ import division
import os
import math

import rospkg
import rospy
from robot_msgs.msg import ChargersMsg, StateMsg
from robot.agent import State
from std_srvs.srv import Empty

from PyQt5 import uic
from PyQt5.QtCore import pyqtSlot as Slot
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QWidget, QTableWidgetItem, QTableWidget, QAbstractScrollArea, QAbstractItemView


class AgentDashboardWidget(QWidget):
    """
    main class inherits from the ui window class.
    You can specify the topics that the topic pane.
    AgentDashboardWidget.start must be called in order to update topic pane.
    """
    def __init__(self, plugin=None):

        super(AgentDashboardWidget, self).__init__()
        
        self._states = {State.IDLE: 'idle', State.MOVING: 'moving', State.CHARGING: 'charging'} 
        self._topics = {'chargers': '/ml/chargers', 'agent_state': '/ml/agent_state'}

        rp = rospkg.RosPack()
        ui_file_path = os.path.join(rp.get_path('rqt_plugins'), 'resource', 'AgentDashboard.ui')
        uic.loadUi(ui_file_path, self)
        self._tableChargersTopic: QTableWidget = self.tableChargersTopic
        self._tableAgentState: QTableWidget = self.tableAgentState

        rospy.wait_for_service('/gazebo/pause_physics')
        self._pause_sim_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        rospy.wait_for_service('/gazebo/unpause_physics')
        self._unpause_sim_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        
        self.pauseSim.clicked.connect(self.pause_simulation)
        self.unpauseSim.clicked.connect(self.unpause_simulation)

        # self._plugin = plugin 

        # init and start update timer
        self._timer_refresh_chargers_topic = QTimer(self)
        self._timer_refresh_chargers_topic.timeout.connect(self.refresh_chargers_topic)

        self._timer_agent_state_topic = QTimer(self)
        self._timer_agent_state_topic.timeout.connect(self.refresh_agent_state_topic)

    def start(self):
        """
        This method needs to be called to start updating topic pane.
        """
        self._timer_refresh_chargers_topic.start(500)
        self._timer_agent_state_topic.start(500)

    @Slot()
    def pause_simulation(self):
        self._pause_sim_client()

    @Slot()
    def unpause_simulation(self):
        self._unpause_sim_client()

    @Slot()
    def refresh_chargers_topic(self):
        """
        refresh chargers data
        """
        rospy.logdebug(f'AgentDashboardWidget: Waiting for {self._topics["chargers"]} message...')
        try:
            chargers_state = rospy.wait_for_message(self._topics['chargers'], ChargersMsg, rospy.Duration(1))
        except rospy.ROSException as e:
            rospy.loginfo('new data no available...')
            return
        self._tableChargersTopic.setRowCount(len(chargers_state.chargers))
        for i, charger in enumerate(chargers_state.chargers):
            self._tableChargersTopic.setItem(i, 0, QTableWidgetItem(charger.name))
            self._tableChargersTopic.setItem(i, 1, QTableWidgetItem(str(charger.coords.x)))
            self._tableChargersTopic.setItem(i, 2, QTableWidgetItem(str(charger.coords.y)))
            charger_value_item = QTableWidgetItem(f'{charger.charger_value:.4f}')
            if charger.charger_value < 0.1:
                charger_value_item.setBackground(Qt.red)
            elif charger.charger_value < 0.5:
                charger_value_item.setBackground(Qt.yellow)
            else:
                charger_value_item.setBackground(Qt.green)
            self._tableChargersTopic.setItem(i, 3, charger_value_item)
        self._tableChargersTopic.resizeColumnsToContents()
        self._tableChargersTopic.resizeRowsToContents()

    @Slot()
    def refresh_agent_state_topic(self):
        """
        refresh agent state data
        """
        rospy.logdebug(f'AgentDashboardWidget: Waiting for {self._topics["agent_state"]} message...')
        try:
            agent_state = rospy.wait_for_message(self._topics['agent_state'], StateMsg, rospy.Duration(1))
        except rospy.ROSException as e:
            rospy.loginfo('new data no available...')
            return
        self._tableAgentState.setRowCount(1)
        self._tableAgentState.setItem(0, 0, QTableWidgetItem(f'{agent_state.coords.x:.3f}'))
        self._tableAgentState.setItem(0, 1, QTableWidgetItem(f'{agent_state.coords.y:.3f}'))
        yaw_degrees = agent_state.yaw * 180 / math.pi
        self._tableAgentState.setItem(0, 2, QTableWidgetItem(f'{int(yaw_degrees)}'))
        battery_level_item = QTableWidgetItem(f'{agent_state.battery_level:.3f}')
        if agent_state.battery_level < 0.1:
            battery_level_item.setBackground(Qt.red)
        elif agent_state.battery_level < 0.5:
            battery_level_item.setBackground(Qt.yellow)
        else:
            battery_level_item.setBackground(Qt.green)
        self._tableAgentState.setItem(0, 3, battery_level_item)
        self._tableAgentState.setItem(0, 4, QTableWidgetItem(str(agent_state.state)))
        self._tableAgentState.setItem(0, 5, QTableWidgetItem(self._states[agent_state.state]))
        self._tableAgentState.resizeColumnsToContents()
        self._tableAgentState.resizeRowsToContents()

    def shutdown_plugin(self):
        self._timer_refresh_chargers_topic.stop()
        self._timer_agent_state_topic.stop()
        self._pause_sim_client.close()
        self._unpause_sim_client.close()
        rospy.logdebug('AgentDashboardWidget: shutdown_plugin')

    def save_settings(self, plugin_settings, instance_settings):
        rospy.logdebug('AgentDashboardWidget: save_settings')

    def restore_settings(self, plugin_settings, instance_settings):
        rospy.logdebug('AgentDashboardWidget: restore_settings')
