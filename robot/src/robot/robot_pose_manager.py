#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from robot_msgs.srv import IsMoving, IsMovingResponse, IsMovingRequest, PoseAndGridCoordsMsg
import math
import tf
import copy


class RobotPoseManagerClient:
    def __init__(self):
        self._service_client = rospy.ServiceProxy('is_moving', IsMoving)
        self._pose_and_grid_cell_coord_sub = rospy.Subscriber('pose_and_grid_coord', PoseAndGridCoordsMsg, self._pose_and_grid_coord_cb, queue_size=1)
        self._pose_and_grid_coords = PoseAndGridCoordsMsg()

    def _pose_and_grid_coord_cb(self, data: PoseAndGridCoordsMsg)
        self._pose_and_grid_coords = data

    def is_moving(self):
        res: IsMovingResponse = self._service_client()
        return res.is_moving

    @property
    def pose_and_grid_coords(self) -> PoseAndGridCoordsMsg:
        return self._pose_and_grid_coords

    @property
    def coords(self) -> Point:
        return self._pose_and_grid_coords.coords

    @property
    def pose(self) -> Point:
        return self._pose_and_grid_coords.pose


class RobotPoseManager:
    def __init__(self):
        self._pose_and_grid_cell = PoseAndGridCoordsMsg()
        self._prev_pose_and_grid_cell = PoseAndGridCoordsMsg()

        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_cb, queue_size=1)
        self._pose_and_grid_cell_coord_pub = rospy.Publisher('pose_and_grid_coord', PoseAndGridCoordsMsg, queue_size=1)
        self._service_is_moving = rospy.Service('is_moving', IsMoving, _is_moving_service_handler)

    def _odom_cb(self, data: Odometry):
        x_pos, y_pos = data.pose.pose.position.x, data.pose.pose.position.y
        quat = data.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self._pose_and_grid_cell.coords = Point(x=math.floor(x_pos + 0.5), y=math.floor(y_pos + 0.5))
        self._pose_and_grid_cell.pose = Point(x=x_pos, y=y_pos, z=yaw)
        continuous_movement = rospy.get_param('/continuous_movement', False)
        if continuous_movement:
            x_changed = abs(self._pose_and_grid_cell.pose.x - self._prev_pose_and_grid_cell.pose.x) > 0.001
            y_changed = abs(self._pose_and_grid_cell.pose.y - self._prev_pose_and_grid_cell.pose.y) > 0.001
            yaw_changed = abs(self._pose_and_grid_cell.pose.z - self._prev_pose_and_grid_cell.pose.z) > 0.01
            self._is_moving = x_changed or y_changed or yaw_changed
        else:
            self._is_moving = self._pose_and_grid_cell.coords != self._prev_pose_and_grid_cell.coords 
        self._pose_and_grid_cell_coord_pub.publish(self._pose_and_grid_cell)

    def step(self):
        self._prev_pose = copy.deepcopy(self._pose)
        self._prev_coords = copy.deepcopy(self._coords)

    def _is_moving_service_handler(self, req: IsMovingRequest) -> IsMovingResponse:
        res = IsMovingResponse()
        res.is_moving = self._is_moving
        return res
