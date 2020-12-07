import rospy
import rospkg
from robot_msgs.msg import FieldMsg, MapMsg
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
import random
import os
import sys
import time
import numpy as np
from typing import List


class Field:
    NORMAL = 0
    CHARGER = 1
    WHEEL_LUBRICATION = 2

    def __init__(self, x: int, y: int, field_type: int, name: str, nr_visits: int = 0):
        self._x = x
        self._y = y
        self._nr_visits = nr_visits
        self._field_type = field_type
        self._name = name

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

    @property
    def name(self) -> str:
        return self._name

    def increment_nr_visits(self, increment_val = 1):
        self._nr_visits += increment_val

    def get_field_msg(self) -> FieldMsg:
        return FieldMsg(coords=Point(x=self.x, y=self.y), nr_visits=self.nr_visits, type=self.field_type, name=self.name)


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
        self._visualization_markers_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, latch=True, queue_size=1)
        
    @property
    def world_map(self) -> List[Field]:
        return self._world_map

    @staticmethod
    def get_sdf(model_name: str):
        pkg_path = rospkg.RosPack().get_path('turtlebot3_gazebo')
        path = os.path.join(pkg_path, 'models', model_name, 'model.sdf')
        with open(path, 'r') as f:
            model_sdf = f.read()
        return model_sdf

    def remove_models(self):
        rospy.loginfo('Removing old fields...')
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        model_names = model_states.name
        for name in model_names:
            if 'charger' in name or 'wheels_lubrication' in name:
                delete_model(name)

    def create_map(self):
        self.remove_models()

        rospy.loginfo('Waiting for /gazebo/spawn_sdf_model service...')
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        models_sdf = {Field.NORMAL: GlobalMapManager.get_sdf('normal_field'), 
                      Field.CHARGER: GlobalMapManager.get_sdf('charger'),
                      Field.WHEEL_LUBRICATION: GlobalMapManager.get_sdf('wheels_lubrication')}

        width = 11
        height = 11
        nr_fields = width * height
        normal_prob = 1 - self._chargers_prob - self._wheels_lubrication_prob
        pkg_path = rospkg.RosPack().get_path('environment')
        try:
            map_fields = np.loadtxt(os.path.join(pkg_path, 'data', 'map_fields.txt'), dtype=int)
        except Exception as e:
            rospy.loginfo('File with map fields types not available. Generating randomly...')
            map_fields = np.random.choice([0, 1, 2], (height, width), p=[normal_prob, self._chargers_prob, self._wheels_lubrication_prob])
            np.savetxt(os.path.join(pkg_path, 'data', 'map_fields.txt'), map_fields, fmt='%d')
        
        cnts = {Field.CHARGER: 0, Field.WHEEL_LUBRICATION: 0, Field.NORMAL: 0}
        # names = {Field.CHARGER: 'charger', Field.WHEEL_LUBRICATION: 'wheels_lubrication', Field.NORMAL: 'normal'}
        for i in range(map_fields.shape[0]):
            for j in range(map_fields.shape[1]):
                field_type = map_fields[i, j]
                if i == map_fields.shape[0] // 2 and j == map_fields.shape[1] // 2:
                    field_type = Field.NORMAL
                field = Field(x=int(i - map_fields.shape[0] // 2), y=int(j - map_fields.shape[1] // 2),
                              field_type=field_type,
                              name=f'{GlobalMapManager.FIELD_NAMES[field_type]}-{cnts[field_type]}')
                self._world_map.append(field)

                # Spawning in gazebo part
                if field.field_type == Field.NORMAL:
                    # Spawn model only for non normal fields (to save time)
                    cnts[Field.NORMAL] += 1
                    continue

                # pose = Pose()
                # pose.position.x = field.x
                # pose.position.y = field.y
                # req = SpawnModelRequest()
                # req.model_name = f'{GlobalMapManager.FIELD_NAMES[field.field_type]}-{cnts[field.field_type]}'
                # req.model_xml = models_sdf[field.field_type]
                # req.initial_pose = pose
                # req.reference_frame = 'world'
                # spawn_sdf_model.call(req)

                cnts[field.field_type] += 1
        self.publish()
        self.publish_marker_array()

    def publish(self):
        global_map_msg = MapMsg()
        global_map_msg.header.stamp = rospy.Time().now()
        for field in self._world_map:
            global_map_msg.fields.append(field.get_field_msg())
        self._world_map_pub.publish(global_map_msg)

    def publish_marker_array(self):
        try:
            marker_array = MarkerArray()
            cnts = {Field.CHARGER: 0, Field.WHEEL_LUBRICATION: 0, Field.NORMAL: 0}
            # names = {Field.CHARGER: 'charger', Field.WHEEL_LUBRICATION: 'wheels_lubrication', Field.NORMAL: 'normal'}
            for field in self._world_map:
                marker = Marker()
                marker.header.stamp = rospy.Time().now()
                marker.header.frame_id = 'odom'
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position = Point(x=field.x, y=field.y, z=-0.01)
                marker.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
                marker.scale = Vector3(x=0.95, y=0.95, z=0.005)
                marker.ns = GlobalMapManager.FIELD_NAMES[field.field_type]
                marker.id = cnts[field.field_type]
                cnts[field.field_type] += 1
                if field.field_type == Field.CHARGER:
                    marker.color = ColorRGBA(r=0, g=1, b=0, a=1)
                elif field.field_type == Field.WHEEL_LUBRICATION:
                    marker.color = ColorRGBA(r=1, g=0.5, b=0, a=1)
                elif field.field_type == Field.NORMAL:
                    marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1)
                marker.lifetime = rospy.Duration()
                marker_array.markers.append(marker)
            self._visualization_markers_pub.publish(marker_array)
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)
            raise
