#! /usr/bin/env python3
import sys
import rospy
import actionlib
from geometry_msgs.msg import Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from robot_msgs.srv import SetGridGoal, SetGridGoalRequest, SetGridGoalResponse
from robot_msgs.srv import CancelGridGoal, CancelGridGoalRequest, CancelGridGoalResponse
import math
import tf


class GoToGridCellService:
    def __init__(self, continuous_movement=False):
        self.continuous_movement = continuous_movement
        if self.continuous_movement:
            self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            rospy.loginfo('GoToGridCellService: waiting for move base server...')
            self.move_base_client.wait_for_server()
            rospy.loginfo('GoToGridCellService: move base server available')
        else:
            self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState) 
            rospy.loginfo('GoToGridCellService: waiting for /gazebo/set_model_state service...')
            self.set_model_state.wait_for_service()
        rospy.loginfo('GoToGridCellService: /gazebo/set_model_state service available')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_grid_goal_server = rospy.Service('set_grid_goal', SetGridGoal, self.execute_cb)
        self.cancel_grid_goal_server = rospy.Service('cancel_grid_goal', CancelGridGoal, self.cancel_cb)
        self.model_name = rospy.get_param('model_name', 'turtlebot3_waffle')

    def cancel_cb(self, req: CancelGridGoalRequest):
        rospy.loginfo('Canceling goal')
        if self.continuous_movement:
            self.move_base_client.cancel_all_goals()
        return CancelGridGoalResponse()

    def execute_cb(self, grid_goal: SetGridGoalRequest):
        rospy.loginfo(f'Executing, creating go to grid cell:\n{grid_goal}')
        if self.continuous_movement:
            goal_world = grid_goal.goal

            pos, _ = self.get_current_pose_quaternion()
            inc_x = goal_world.x - pos.x
            inc_y = goal_world.y - pos.y
            angle_to_goal = math.atan2(inc_y, inc_x)

            g_orien = tf.transformations.quaternion_from_euler(0, 0, angle_to_goal)
            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose.header.frame_id = "odom"
            move_base_goal.target_pose.header.stamp = rospy.Time.now()
            move_base_goal.target_pose.pose.position = goal_world
            move_base_goal.target_pose.pose.orientation = Quaternion(x=g_orien[0], y=g_orien[1], z=g_orien[2], w=g_orien[3])
            self.move_base_client.send_goal(move_base_goal)
        else:
            model_state = ModelState()
            model_state.model_name = self.model_name       
            model_state.pose.position.x = grid_goal.goal.x
            model_state.pose.position.y = grid_goal.goal.y
            self.set_model_state(model_state)
        return SetGridGoalResponse()

    def get_current_pose_quaternion(self):
        res = self.get_model_state(self.model_name, '')
        return res.pose.position, res.pose.orientation


if __name__ == '__main__':
    try:
        rospy.init_node('go_to_grid_cell_service', log_level=rospy.INFO)
        continuous_movement = rospy.get_param('~continuous_movement', True)
        go_to_grid_cell = GoToGridCellService(continuous_movement=continuous_movement)
        rospy.loginfo('Go to grid service running...')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
