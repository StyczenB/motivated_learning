#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math
import tf
import copy


class RobotPoseManager:
    def __init__(self):
        self._coords = Point()
        self._prev_coords = Point()
        self._pose = Point()
        self._prev_pose = Point()
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_cb, queue_size=1)
        self._grid_cell_coord_pub = rospy.Publisher('grid_coord', Point, queue_size=1)

    def _odom_cb(self, data: Odometry):
        x_pos, y_pos = data.pose.pose.position.x, data.pose.pose.position.y
        quat = data.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self._coords = Point(x=math.floor(x_pos + 0.5), y=math.floor(y_pos + 0.5))
        self._pose = Point(x=x_pos, y=y_pos, z=yaw)
        self._grid_cell_coord_pub.publish(self._coords)

    @property
    def coords(self) -> Point:
        return self._coords

    @property
    def pose(self) -> Point:
        return self._pose

    def step(self):
        self._prev_pose = copy.deepcopy(self._pose)
        self._prev_coords = copy.deepcopy(self._coords)

    def is_moving(self) -> bool:
        continuous_movement = rospy.get_param('/continuous_movement', True)
        if continuous_movement:
            x_changed = abs(self._pose.x - self._prev_pose.x) > 0.001
            y_changed = abs(self._pose.y - self._prev_pose.y) > 0.001
            yaw_changed = abs(self._pose.z - self._prev_pose.z) > 0.01
            return x_changed or y_changed or yaw_changed
        else:
            return self._coords != self._prev_coords 
