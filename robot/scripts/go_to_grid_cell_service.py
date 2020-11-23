#! /usr/bin/env python3
import sys
import rospy
import actionlib
from geometry_msgs.msg import Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import GetModelState
from robot_msgs.srv import SetGridGoal, SetGridGoalRequest, SetGridGoalResponse, CancelGridGoal, CancelGridGoalRequest, CancelGridGoalResponse
import math
import tf


class GoToGridCellService:
    def __init__(self):
        self.move_base_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.get_model_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)

        self.set_grid_goal_server = rospy.Service(
            'set_grid_goal', SetGridGoal, self.execute_cb)
        self.cancel_grid_goal_server = rospy.Service(
            'cancel_grid_goal', CancelGridGoal, self.cancel_cb)

    def cancel_cb(self, req: CancelGridGoalRequest):
        rospy.loginfo('Canceling goal')
        self.move_base_client.cancel_all_goals()
        return CancelGridGoalResponse()

    def execute_cb(self, grid_goal: SetGridGoalRequest):
        rospy.loginfo(f'Executing, creating go to grid cell:\n{grid_goal}')
        goal_world = GoToGridCellService.get_world_from_grid_coordinate(grid_goal.goal)

        pos, _ = self.get_current_pose_quaternion()
        inc_x = goal_world.x - pos.x
        inc_y = goal_world.y - pos.y
        angle_to_goal = math.atan2(inc_y, inc_x)

        g_orien = tf.transformations.quaternion_from_euler(0, 0, angle_to_goal)
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "odom"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position = goal_world
        move_base_goal.target_pose.pose.orientation = Quaternion(
            x=g_orien[0], y=g_orien[1], z=g_orien[2], w=g_orien[3])
        self.move_base_client.send_goal(move_base_goal)
        return SetGridGoalResponse()

    @staticmethod
    def get_world_from_grid_coordinate(grid: Point):
        x_pos = grid.x + 0.5
        y_pos = grid.y + 0.5
        return Point(x=x_pos, y=y_pos, z=0)

    def get_current_pose_quaternion(self):
        res = self.get_model_state('turtlebot3_waffle', '')
        return res.pose.position, res.pose.orientation


if __name__ == '__main__':
    try:
        rospy.init_node('go_to_grid_cell_service')
        go_to_grid_cell = GoToGridCellService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
