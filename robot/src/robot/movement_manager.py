#! /usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Quaternion, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from robot_msgs.srv import SetGridGoal, SetGridGoalRequest, SetGridGoalResponse
from robot_msgs.srv import CancelGridGoal, CancelGridGoalRequest, CancelGridGoalResponse
from robot_msgs.msg import PoseAndGridCoordsMsg
import math
import tf
import threading
import copy


class MovementManagerClient:
    def __init__(self):
        self._set_grid_goal_client = rospy.ServiceProxy('set_grid_goal', SetGridGoal)
        self._set_grid_goal_client.wait_for_service()
        self._cancel_grid_goal_client = rospy.ServiceProxy('cancel_grid_goal', CancelGridGoal)
        self._cancel_grid_goal_client.wait_for_service()

    def send_goal(self, x: int, y: int, local=True, continuous_movement=False):
        self._set_grid_goal_client(Point(x=x, y=y), local, continuous_movement)

    def cancel_goal(self):
        self._cancel_grid_goal_client()


class MovementManager:
    def __init__(self):
        # self._continuous_movement = rospy.get_param('/continuous_movement', True)
        self._model_name = rospy.get_param('/model_name', 'turtlebot3_waffle')
        # if self._continuous_movement:
        self._move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()
        # else:
        self._set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self._set_model_state.wait_for_service()
        rospy.loginfo('MovementManager: /gazebo/set_model_state service available')
        self._get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self._set_grid_goal_server = rospy.Service('set_grid_goal', SetGridGoal, self.execute_cb)
        self._cancel_grid_goal_server = rospy.Service('cancel_grid_goal', CancelGridGoal, self.cancel_cb)
        self._current_pose_and_grid_coords = PoseAndGridCoordsMsg()
        self._pose_and_grid_coords_sub = rospy.Subscriber('pose_and_grid_coord', PoseAndGridCoordsMsg, self.pose_and_grid_coords_cb)
        self._lock = threading.Lock()

    def __del__(self):
        self._move_base_client.cancel_all_goals()
        self._set_model_state.close()
        self._get_model_state.close()
        self._set_grid_goal_server.shutdown()
        self._cancel_grid_goal_server.shutdown()
        self._pose_and_grid_coords_sub.unregister()

    def pose_and_grid_coords_cb(self, pose_and_grid_coords: PoseAndGridCoordsMsg):
        with self._lock:
            self._current_pose_and_grid_coords = pose_and_grid_coords

    def cancel_cb(self, req: CancelGridGoalRequest):
        rospy.loginfo('Canceling goal')
        # if self._continuous_movement:
        self._move_base_client.cancel_all_goals()
        return CancelGridGoalResponse()

    def execute_cb(self, grid_goal: SetGridGoalRequest):
        goal_world = grid_goal.goal
        with self._lock:
            current_grid_coords = copy.deepcopy(self._current_pose_and_grid_coords.coords)
        rospy.loginfo(f'Goal:\n{grid_goal}\nCurrent grid coords:\n{current_grid_coords}')
        if current_grid_coords.x == goal_world.x and current_grid_coords.y == goal_world.y:
            return SetGridGoalResponse()
        if grid_goal.local:
            diff_vec = Point(x=goal_world.x - current_grid_coords.x,
                             y=goal_world.y - current_grid_coords.y)
            if abs(diff_vec.x) > abs(diff_vec.y):
                abs_local_goal = Point(x=1, y=0)
            elif abs(diff_vec.x) < abs(diff_vec.y):
                abs_local_goal = Point(x=0, y=1)
            else:
                abs_local_goal = Point(x=1, y=1)
            goal_world.x = current_grid_coords.x + abs_local_goal.x * (1 if diff_vec.x > 0 else -1)
            goal_world.y = current_grid_coords.y + abs_local_goal.y * (1 if diff_vec.y > 0 else -1)
        if grid_goal.continuous_movement:
            g_orien = self.get_goal_orientation(goal_world)
            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose.header.frame_id = "odom"
            move_base_goal.target_pose.header.stamp = rospy.Time.now()
            move_base_goal.target_pose.pose.position = goal_world
            move_base_goal.target_pose.pose.orientation = Quaternion(x=g_orien[0], y=g_orien[1], z=g_orien[2], w=g_orien[3])
            self._move_base_client.send_goal(move_base_goal)
        else:
            model_state = ModelState()
            model_state.model_name = self._model_name
            model_state.pose.position.x = goal_world.x
            model_state.pose.position.y = goal_world.y
            self._set_model_state(model_state)
        return SetGridGoalResponse()

    def get_goal_orientation(self, goal: Point) -> Quaternion:
        pos, _ = self.get_current_pose_quaternion()
        inc_x, inc_y = goal.x - pos.x, goal.y - pos.y
        angle_to_goal = math.atan2(inc_y, inc_x)
        g_orien = tf.transformations.quaternion_from_euler(0, 0, angle_to_goal)
        return g_orien

    def get_current_pose_quaternion(self):
        res = self._get_model_state(self._model_name, '')
        return res.pose.position, res.pose.orientation
