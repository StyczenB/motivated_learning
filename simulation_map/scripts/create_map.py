#!/usr/bin/env python3
import rospy
import rospkg
from robot_msgs.msg import Field, Map
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose
import random
import os
import math
import time

FIELD_TYPES = {'charger': Field.CHARGER,
               'wheels_lubrication': Field.WHEEL_LUBRICATION, 
               'normal_field': Field.NORMAL}
FIELD_NAMES = {v: k for k, v in FIELD_TYPES.items()}


def get_sdf(model_name: str):
    pkg_path = rospkg.RosPack().get_path('turtlebot3_gazebo')
    path = os.path.join(pkg_path, 'models', model_name, 'model.sdf')
    with open(path, 'r') as f:
        model_sdf = f.read()
    return model_sdf


def create_map():
    rospy.loginfo('Waiting for /gazebo/spawn_sdf_model service...')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    models_sdf = {Field.NORMAL: get_sdf('normal_field'), 
                  Field.CHARGER: get_sdf('charger'), 
                  Field.WHEEL_LUBRICATION: get_sdf('wheels_lubrication')}

    width = 11
    height = 11
    nr_fields = width * height
    charger_mod = 8
    wheels_lubrication_mod = 15
    world_map = Map()
    for i in range(nr_fields):
        field = Field()
        field.x = i // width - width // 2
        field.y = i % width - height // 2
        field.nr_visits = 0

        if i % charger_mod == 0:
            field.type = Field.CHARGER
        elif i % wheels_lubrication_mod == 0:
            field.type = Field.WHEEL_LUBRICATION
        else:
            field.type = Field.NORMAL

        # Starting field is always normal
        if i == nr_fields // 2:
            field.type = Field.NORMAL

        world_map.cells.append(field)

        # Spawn model only for non normal fields (to save time)
        if field.type == Field.NORMAL:
            continue

        pose = Pose()
        pose.position.x = field.x + 0.5
        pose.position.y = field.y + 0.5
        req = SpawnModelRequest()
        req.model_name = f'{FIELD_NAMES[field.type]}{i}'
        req.model_xml = models_sdf[field.type]
        req.initial_pose = pose
        req.reference_frame = 'world'
        spawn_sdf_model.call(req)

    return world_map


if __name__ == '__main__':
    try:
        rospy.init_node('create_map')
        world_map_pub = rospy.Publisher('global_world_map', Map, latch=True, queue_size=1)
        start = time.time()
        world_map = create_map()
        stop = time.time()
        rospy.loginfo(f'Creating map execution time: {stop - start} sec.')
        world_map_pub.publish(world_map)
        rospy.loginfo('Map loaded')
        rospy.spin()
    except:
        rospy.logerr('Error while creating map or user exiting...')
    
