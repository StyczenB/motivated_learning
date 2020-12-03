from __future__ import division
import os
import math

import rospkg
import rospy
from robot_msgs.msg import ChargersMsg, StateMsg
from robot.agent import State
from std_srvs.srv import Empty
from robot.movement_manager import MovementManagerClient

from PyQt5 import uic
from PyQt5.QtCore import pyqtSlot as Slot
from PyQt5.QtCore import pyqtSignal as Signal
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QWidget, QTableWidgetItem, QTableWidget, QAbstractScrollArea, QAbstractItemView, QRadioButton, QLineEdit


class AgentDashboardWidget(QWidget):
    """
    main class inherits from the ui window class.
    You can specify the topics that the topic pane.
    AgentDashboardWidget.start must be called in order to update topic pane.
    """

    agent_state_signal = Signal(StateMsg)
    chargers_signal = Signal(ChargersMsg)

    def __init__(self, plugin=None):

        super(AgentDashboardWidget, self).__init__()
        
        self._states = {State.IDLE: 'idle', State.MOVING: 'moving', State.CHARGING: 'charging'} 
        self._topics = {'chargers': '/ml/chargers', 'agent_state': '/ml/agent_state'}

        self._movement_manager = MovementManagerClient()

        rp = rospkg.RosPack()
        ui_file_path = os.path.join(rp.get_path('rqt_plugins'), 'resource', 'AgentDashboard.ui')
        uic.loadUi(ui_file_path, self)

        # Tables with chargers states and agent state
        self._tableChargersTopic: QTableWidget = self.tableChargersTopic
        self._tableAgentState: QTableWidget = self.tableAgentState

        # Sending goal to agent
        self.sendGoal.clicked.connect(self.send_goal)
        self.cancelGoal.clicked.connect(self.cancel_goal)

        # Pausing anf unpausing simulation
        self._pause_sim_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self._pause_sim_client.wait_for_service()
        self._unpause_sim_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self._unpause_sim_client.wait_for_service()
        self.pauseSim.clicked.connect(self.pause_simulation)
        self.unpauseSim.clicked.connect(self.unpause_simulation)

        self.agent_state_signal.connect(self.refresh_agent_state_window)
        self.chargers_signal.connect(self.refresh_chargers_window)

        self.localCheckBox.setChecked(rospy.get_param('/local', True))
        self.continousCheckBox.setChecked(rospy.get_param('/continuous_movement', False))

    def send_goal(self):
        local = self.localCheckBox.isChecked() 
        continous = self.continousCheckBox.isChecked()
        rospy.set_param('/continuous_movement', continous)
        rospy.set_param('/local', local)
        try:
            x_goal = int(self.xCoordEdit.text())
            y_goal = int(self.yCoordEdit.text())    
            self._movement_manager.send_goal(x_goal, y_goal, local, continous)
        except ValueError as e:
            rospy.logwarn(f'send_goal error: {e}')

    def cancel_goal(self):
        self._movement_manager.cancel_goal()
    
    def start(self):
        """
        This method needs to be called to start updating topic pane.
        """
        self._agent_state_sub = rospy.Subscriber(self._topics['agent_state'], StateMsg, self.agent_state_cb)
        self._chargers_sub = rospy.Subscriber(self._topics['chargers'], ChargersMsg, self.chargers_cb)

    def agent_state_cb(self, agent_state: StateMsg):
        self.agent_state_signal.emit(agent_state)

    def chargers_cb(self, chargers_state: ChargersMsg):
        self.chargers_signal.emit(chargers_state)

    @Slot(ChargersMsg)
    def refresh_chargers_window(self, chargers_state: ChargersMsg):
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

    @Slot(StateMsg)
    def refresh_agent_state_window(self, agent_state: StateMsg):
        self._tableAgentState.setRowCount(1)
        content = f'x: {agent_state.pose.x:.2f}\n'
        content += f'y: {agent_state.pose.y:.2f}\n'
        content += f'z: {agent_state.pose.z:.2f}'
        self._tableAgentState.setItem(0, 0, QTableWidgetItem(content))
        self._tableAgentState.setItem(0, 1, QTableWidgetItem(f'{agent_state.coords}'))
        battery_level_item = QTableWidgetItem(f'{agent_state.battery_level:.3f}')
        if agent_state.battery_level < 0.1:
            battery_level_item.setBackground(Qt.red)
        elif agent_state.battery_level < 0.5:
            battery_level_item.setBackground(Qt.yellow)
        else:
            battery_level_item.setBackground(Qt.green)
        self._tableAgentState.setItem(0, 2, battery_level_item)
        self._tableAgentState.setItem(0, 3, QTableWidgetItem(str(agent_state.state)))
        self._tableAgentState.setItem(0, 4, QTableWidgetItem(self._states[agent_state.state]))
        self._tableAgentState.resizeColumnsToContents()
        self._tableAgentState.resizeRowsToContents()
        
    @Slot()
    def pause_simulation(self):
        self._pause_sim_client()

    @Slot()
    def unpause_simulation(self):
        self._unpause_sim_client()

    def shutdown_plugin(self):
        self._pause_sim_client.close()
        self._unpause_sim_client.close()
        self._agent_state_sub.unregister()
        self._chargers_sub.unregister()
        rospy.logdebug('AgentDashboardWidget: shutdown_plugin')

    def save_settings(self, plugin_settings, instance_settings):
        rospy.logdebug('AgentDashboardWidget: save_settings')

    def restore_settings(self, plugin_settings, instance_settings):
        rospy.logdebug('AgentDashboardWidget: restore_settings')
