#!/usr/bin/env python3
import rospy
from robot_msgs.msg import Field, Map
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose
import random
import os
import math
import numpy as np

BASE_DIR = '/home/bartek/study/repo/src'
FIELD_TYPES = {'charger': Field.CHARGER, 'wheels_lubrication': Field.WHEEL_LUBRICATION, 'normal_field': Field.NORMAL}
FIELD_NAMES = {v: k for k, v in FIELD_TYPES.items()}


def get_sdf(model_name: str):
    path = os.path.join(BASE_DIR, 'turtlebot3_gazebo',
                        'models', model_name, 'model.sdf')
    with open(path, 'r') as f:
        model_sdf = f.read()
    return model_sdf


def spawn_fields(model_name: str, x_coords: list, y_coords: list, world_map: Map):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    model_sdf=get_sdf(model_name)
    for i in range(len(x_coords)):
        field=Field()
        field.type=FIELD_TYPES[model_name]
        field.nr_visits=0
        field.x=x_coords[i] + 0.5
        field.y=y_coords[i] + 0.5
        world_map.cells.append(field)
        pose=Pose()
        pose.position.x=field.x
        pose.position.y=field.y

        req=SpawnModelRequest()
        req.model_name=f'{model_name}{i}'
        req.model_xml=model_sdf
        req.initial_pose=pose
        req.reference_frame='world'
        spawn_sdf_model.call(req)


def spawn_field(model_name: str, model_sdf: str, x_coord: int, y_coord: int, world_map: Map, spawn_sdf_model: rospy.ServiceProxy):
    field=Field()
    field.type=FIELD_TYPES[model_name]
    field.nr_visits=0
    field.x = x_coord + 0.5
    field.y = y_coord + 0.5
    world_map.cells.append(field)
    pose=Pose()
    pose.position.x=field.x
    pose.position.y=field.y

    req=SpawnModelRequest()
    req.model_name=f'{model_name}{i}'
    req.model_xml=model_sdf
    req.initial_pose=pose
    req.reference_frame='world'
    spawn_sdf_model.call(req)


def create_map():
    world_map = Map()

    # Prepare coordinates
    nr_fields = 100
    nr_chargers = 5
    nr_wheel_lubrication = 4
    nr_normal_fields = nr_fields - nr_chargers - nr_wheel_lubrication
    x_coords = random.sample(range(-5, 5), int(math.sqrt(nr_fields)))
    y_coords=random.sample(range(-5, 5), int(math.sqrt(nr_fields)))
    x_coords_chargers=x_coords[:nr_chargers]
    y_coords_chargers=y_coords[:nr_chargers]
    x_coords_wheel_lubrications=x_coords[nr_chargers:nr_chargers+nr_wheel_lubrication]
    y_coords_wheel_lubrications=y_coords[nr_chargers:nr_chargers+nr_wheel_lubrication]
    x_coords_normal_fields=x_coords[nr_chargers+nr_wheel_lubrication:]
    y_coords_normal_fields=y_coords[nr_chargers+nr_wheel_lubrication:]

    print(x_coords)
    
    # spawn_fields('charger', x_coords_chargers, y_coords_chargers, world_map)
    # spawn_fields('wheels_lubrication', x_coords_wheel_lubrications, y_coords_wheel_lubrications, world_map)
    # spawn_fields('normal_field', x_coords_normal_fields, y_coords_normal_fields, world_map)

    return world_map


if __name__ == '__main__':
    rospy.init_node('create_map')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    models_sdf = {Field.NORMAL: get_sdf('normal_field'), Field.CHARGER: get_sdf('charger'), Field.WHEEL_LUBRICATION: get_sdf('wheels_lubrication')} 

    # nr_fields = 100
    # nr_chargers = 5
    # nr_wheel_lubrication = 4
    # nr_normal_fields = nr_fields - nr_chargers - nr_wheel_lubrication
    # x_coords = random.sample(range(-5, 5), int(math.sqrt(nr_fields)))
    # y_coords=random.sample(range(-5, 5), int(math.sqrt(nr_fields)))
    # print(list(zip(x_coords, y_coords)))





    # zvals = np.random.rand(10, 10)*3
    # zvals = np.uint8(zvals)
    # print(zvals)
    # world_map=create_map()
    # cnt = 0
    # for i in range(zvals.shape[0]):
    #     for j in range(zvals.shape[1]):
    #         field_type_idx = zvals[i, j]
    #         field=Field()
    #         field.type=field_type_idx
    #         field.nr_visits=0
    #         field.x = (i - zvals.shape[0] / 2) + 0.5
    #         field.y = (j - zvals.shape[1] / 2) + 0.5
    #         world_map.cells.append(field)
    #         pose=Pose()
    #         pose.position.x=field.x
    #         pose.position.y=field.y

    #         req=SpawnModelRequest()
    #         req.model_name=f'{FIELD_NAMES[field_type_idx]}{cnt}'
    #         req.model_xml=models[field_type_idx]
    #         req.initial_pose=pose
    #         req.reference_frame='world'
    #         spawn_sdf_model.call(req)
    #         cnt += 1

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

        if field.type == Field.NORMAL:
            continue
        
        pose = Pose()
        pose.position.x = field.x + 0.5
        pose.position.y = field.y + 0.5
        req=SpawnModelRequest()
        req.model_name = f'{FIELD_NAMES[field.type]}{i}'
        req.model_xml = models_sdf[field.type]
        req.initial_pose = pose
        req.reference_frame = 'world'
        spawn_sdf_model.call(req)

    rospy.loginfo('Map created')
    world_map_pub=rospy.Publisher('/world_map', Map, latch=True, queue_size=1)
    world_map_pub.publish(world_map)
    rospy.spin()
