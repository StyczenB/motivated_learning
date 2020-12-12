#!/usr/bin/env python
"""
This class should take information about current battery level of robot
and calculate its pain associated with battery charge level.

For curiosity it should get current map from robot's memory and 
"""
import rospy
from robot_msgs import AgentStateMsg


class Pains:
    def __init__(self):
        self._curiosity = 0
        self._low_battery_level = 0
        self._necesity_of_wheel_lubrication = 0
        self._homesick = 0
        self._agent_state_sub = rospy.Subscriber('agent_state', AgentStateMsg, self._agent_state_cb, queue_size=1)

    def _agent_state_cb(self):
        pass

    @property
    def curiosity(self):
        return self._curiosity

    @property
    def low_battery_level(self):
        return self._low_battery_level

    @property
    def necesity_of_wheel_lubrication(self):
        return self._necesity_of_wheel_lubrication

    @property
    def homesick(self):
        return self._homesick

    def step(self):
        pass
