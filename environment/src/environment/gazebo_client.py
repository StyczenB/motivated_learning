#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetWorldProperties, GetWorldPropertiesResponse
from gazebo_msgs.srv import GetModelState, GetModelStateResponse


class GazeboClient:
    @staticmethod
    def get_world_properties() -> GetWorldPropertiesResponse:
        world_properties_client = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        world_properties_client.wait_for_service()
        res = world_properties_client()
        return res

    @staticmethod
    def get_sim_time() -> float:
        world_properties_client = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        world_properties_client.wait_for_service()
        properties: GetWorldPropertiesResponse = world_properties_client()
        return properties.sim_time

    @staticmethod
    def get_model_state(model_name: str, reference_frame: str = '') -> GetModelStateResponse:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        get_model_state.wait_for_service()
        res: GetModelStateResponse = get_model_state(model_name, reference_frame)
        return res
