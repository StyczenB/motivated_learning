#!/usr/bin/env python3
import rospy
import rospkg
from robot_msgs.msg import FieldMsg, MapMsg
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose
import random
import os
import time
import numpy as np
from typing import List


class Field:
    NORMAL = 0
    CHARGER = 1
    WHEEL_LUBRICATION = 2

    def __init__(self, x: int, y: int, field_type: int, nr_visits: int = 0):
        self._x = x
        self._y = y
        self._nr_visits = nr_visits
        self._field_type = field_type

    @property
    def x(self) -> int:
        return self._x

    @property
    def y(self) -> int:
        return self._y

    @property
    def nr_visits(self) -> int:
        return self._nr_visits

    @property
    def field_type(self) -> int:
        return self._field_type

    def increment_nr_visits(self, increment_val = 1):
        self._nr_visits += increment_val

    def get_field_msg(self) -> FieldMsg:
        return FieldMsg(x=self._x, y=self._y, nr_visits=self._nr_visits, type=self._type)


class GlobalMapManager:
    FIELD_TYPES = {'charger': Field.CHARGER,
                   'wheels_lubrication': Field.WHEEL_LUBRICATION, 
                   'normal_field': Field.NORMAL}
    FIELD_NAMES = {v: k for k, v in FIELD_TYPES.items()}

    def __init__(self, chargers_prob=0.1, wheels_lubrication_prob=0.05):
        self._chargers_prob = chargers_prob
        self._wheels_lubrication_prob = wheels_lubrication_prob
        self._world_map: List[Field] = []
        self._world_map_pub = rospy.Publisher('global_world_map', MapMsg, latch=True, queue_size=1)
        
    @staticmethod
    def get_sdf(model_name: str):
        pkg_path = rospkg.RosPack().get_path('turtlebot3_gazebo')
        path = os.path.join(pkg_path, 'models', model_name, 'model.sdf')
        with open(path, 'r') as f:
            model_sdf = f.read()
        return model_sdf

    def create_map(self):
        rospy.loginfo('Waiting for /gazebo/spawn_sdf_model service...')
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        models_sdf = {Field.NORMAL: GlobalMapManager.get_sdf('normal_field'), 
                      Field.CHARGER: GlobalMapManager.get_sdf('charger'), 
                      Field.WHEEL_LUBRICATION: GlobalMapManager.get_sdf('wheels_lubrication')}

        width = 11
        height = 11
        nr_fields = width * height
        normal_prob = 1 - self.chargers_prob - self.wheels_lubrication_prob
        pkg_path = rospkg.RosPack().get_path('environment')
        try:
            coord = np.loadtxt(os.path.join(pkg_path, 'data', 'map_fields.txt'), dtype=int)
        except Exception as e:
            rospy.loginfo('File with map fields types not available. Generating randomly...')
            coord = np.random.choice([0, 1, 2], (height, width), p=[normal_prob, self.chargers_prob, self.wheels_lubrication_prob])
            np.savetxt(os.path.join(pkg_path, 'data', 'map_fields.txt'), coord, fmt='%d')
        cnt = 0
        for i in range(coord.shape[0]):
            for j in range(coord.shape[1]):
                field_type = coord[i, j]
                if i == coord.shape[0] // 2 and j == coord.shape[1] // 2: 
                    field_type = Field.NORMAL
                field = Field(x=int(i - coord.shape[0] // 2), y=int(j - coord.shape[1] // 2), field_type=field_type)
                self._world_map.append(field)

                # Spawning in gazebo part
                if field.type == Field.NORMAL:
                    # Spawn model only for non normal fields (to save time)
                    continue

                pose = Pose()
                pose.position.x = field.x + 0.5
                pose.position.y = field.y + 0.5
                req = SpawnModelRequest()
                req.model_name = f'{GlobalMapManager.FIELD_NAMES[field.type]}{cnt}'
                req.model_xml = models_sdf[field.type]
                req.initial_pose = pose
                req.reference_frame = 'world'
                spawn_sdf_model.call(req)
                cnt += 1
        self.publish()

    def publish(self):
        global_map_msg = MapMsg()
        global_map_msg.header.stamp = rospy.Time().now()
        for field in self._world_map:
            global_map_msg.fields.append(field.get_field_msg())


if __name__ == '__main__':
    try:
        rospy.init_node('create_map')
        global_map_manager = GlobalMapManager()

        start = time.time()
        global_map_manager.create_map()
        stop = time.time()
        rospy.loginfo(f'Creating map execution time: {stop - start} sec.')

        rospy.loginfo('Map loaded')
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Error while creating map with message: {e}')
    
