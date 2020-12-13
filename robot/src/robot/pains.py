#!/usr/bin/env python
"""
This class should take information about current battery level of robot
and calculate its pain associated with battery charge level.

For curiosity it should get current map from robot's memory and 
"""
import time
import rospy
from robot_msgs.msg import AgentStateMsg, PainsMsg, MapMsg, FieldMsg
from environment.gazebo_client import GazeboClient


class Pains:
    """
    Pain class
    """
    
    def __init__(self):
        self._pains_pub = rospy.Publisher('pains', PainsMsg, queue_size=1, latch=True)
        self._agent_state_sub = rospy.Subscriber('agent_state', AgentStateMsg, self._agent_state_cb, queue_size=1)
        self._agent_state = AgentStateMsg()

    def __del__(self):
        self._agent_state_sub.unregister()

    def _agent_state_cb(self, agent_state: AgentStateMsg):
        self._agent_state = agent_state
        
    def step(self) -> PainsMsg:
        """
        Calculates pain values based on agent state.
        Publish them to 'pains' and returns to user.
        """
        pains = PainsMsg()
        pains.header.stamp = rospy.Time.from_sec(GazeboClient.get_sim_time())
        pains.low_battery_level = (AgentStateMsg.BATTERY_MAX - self._agent_state.battery_level) ** 3
        pains.condition_of_wheels = self._agent_state.wheel_lubrication_counter
        pains.homesickness = GazeboClient.get_sim_time() - self._agent_state.last_home_visit  # TODO: Should be scaled by some factor
        pains.curiosity = self._curiosity_pain(self._agent_state.internal_map)
        self._pains_pub.publish(pains)
        return pains

    def _curiosity_pain(self, internal_map: MapMsg) -> float:
        if len(internal_map.fields) == 0:
            return 1
        nr_visits_to_new_fields = sum(map(lambda field: field.nr_visits if field.nr_visits < 3 else 0, internal_map.fields))
        nr_visits_to_all_fields = sum([field.nr_visits for field in internal_map.fields])
        return 1 - nr_visits_to_new_fields / nr_visits_to_all_fields  # FIXME: Probably needs debugging
