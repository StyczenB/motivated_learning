#!/usr/bin/env python
"""
This class should take information about current battery level of robot
and calculate its pain associated with battery charge level.

For curiosity it should get current map from robot's memory and 
"""
import copy
import rospy
from robot_msgs.msg import AgentStateMsg, PainsMsg, MapMsg, FieldMsg
from environment.gazebo_client import GazeboClient


class Pains:
    """
    Pain class
    """

    PAIN_NAMES = {name: idx - 1 for idx, name in enumerate(PainsMsg.__slots__) if name != 'header'}
    PAIN_IDX = {idx: name for name, idx in PAIN_NAMES.items()}
    
    def __init__(self):
        self._pains_pub = rospy.Publisher('pains', PainsMsg, queue_size=1, latch=True)
        self._agent_state_sub = rospy.Subscriber('agent_state', AgentStateMsg, self._agent_state_cb, queue_size=1)
        self._agent_state = AgentStateMsg()
        self._prev_agent_state = AgentStateMsg()
        self._last_internal_map_changed = GazeboClient.get_sim_time()

    def __del__(self):
        self._agent_state_sub.unregister()

    def _agent_state_cb(self, agent_state: AgentStateMsg):
        self._agent_state = agent_state
        
    def step(self) -> (PainsMsg, (int, str, float)):
        """
        Calculates pain values based on agent state.
        Publish them to 'pains' and returns to user.
        """
        pains = PainsMsg()
        pains.header.stamp = rospy.Time.from_sec(GazeboClient.get_sim_time())
        pains.low_battery_level = (AgentStateMsg.BATTERY_MAX - self._agent_state.battery_level) ** 3
        pains.condition_of_wheels = self._agent_state.wheel_lubrication_counter
        pains.homesickness = 0.1 * (GazeboClient.get_sim_time() - self._agent_state.last_home_visit)  # TODO: Should be scaled by some factor
        pains.curiosity = self._curiosity_pain(self._agent_state.internal_map)
        dominant_pain = Pains._get_dominant_pain(pains)
        self._pains_pub.publish(pains)
        self._prev_agent_state = copy.deepcopy(self._agent_state)
        return pains, dominant_pain

    @staticmethod
    def _get_dominant_pain(pains: PainsMsg) -> (int, str, float):
        pains_values = [pains.__getattribute__(pain_name) for pain_name in Pains.PAIN_NAMES.keys()]
        dominant_pain_val = max(pains_values)
        dominant_pain_idx = pains_values.index(dominant_pain_val)
        return {'idx': dominant_pain_idx, 'name': Pains.PAIN_IDX[dominant_pain_idx], 'val': dominant_pain_val}

    def _curiosity_pain(self, internal_map: MapMsg) -> float:
        if not self.internal_map_changed(internal_map):
            return GazeboClient.get_sim_time() - self._last_internal_map_changed
        self._last_internal_map_changed = GazeboClient.get_sim_time()
        nr_visits_to_new_fields = sum(map(lambda field: field.nr_visits if field.nr_visits < 3 else 0, internal_map.fields))
        nr_visits_to_all_fields = sum([field.nr_visits for field in internal_map.fields])
        return 1 - nr_visits_to_new_fields / nr_visits_to_all_fields  # FIXME: Probably needs debugging

    def internal_map_changed(self, current_internal_map: MapMsg):
        return len(current_internal_map.fields) != len(self._prev_agent_state.internal_map.fields)
