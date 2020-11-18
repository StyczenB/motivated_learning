#! /usr/bin/env python
import rospy
import actionlib
from robot_msgs.msg import GoToGridCellAction, GoToGridCellGoal, GoToGridCellFeedback, GoToGridCellResult
from geometry_msgs.msg import Point, Twist
from gazebo_msgs.srv import GetModelState
import tf
import math


class GoToGridCell:
    # create messages that are used to publish feedback/result
    # _feedback = actionlib_tutorials.msg.FibonacciFeedback()
    # _result = actionlib_tutorials.msg.FibonacciResult()

    def __init__(self):
        self._action_name = 'go_to_grid_cell'
        self._as = actionlib.SimpleActionServer(
            self._action_name, GoToGridCellAction, execute_cb=self.execute_cb, auto_start=False)
        self._feedback = GoToGridCellFeedback()
        self._result = GoToGridCellResult()
        self._cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._get_model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)
        self._as.start()
        rospy.loginfo("Initalize go to grid cell action server.")

    def __del__(self):
        speed = Twist()
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        self._cmd_vel_pub.publish(speed)

    def execute_cb(self, goal: GoToGridCellGoal):
        # helper variables
        r = rospy.Rate(5)
        success = False

        goal_world = GoToGridCell.get_world_from_grid_coordinate(
            goal.grid_coordinates)

        # publish info to the console for the user
        rospy.loginfo(
            f'{self._action_name}: Executing, creating go to grid cell action')

        speed = Twist()
        while not success:
            res = self._get_model_state("turtlebot3_waffle", '')
            pos = (res.pose.position.x, res.pose.position.y)
            quat = res.pose.orientation
            _, _, yaw = tf.transformations.euler_from_quaternion(
                [quat.x, quat.y, quat.z, quat.w])

            inc_x = goal_world.x - pos[0]
            inc_y = goal_world.y - pos[1]
            angle_to_goal = math.atan2(inc_y, inc_x)

            dist_from_goal = math.sqrt(inc_x**2 + inc_y**2)
            self._feedback.dist_to_goal = dist_from_goal
            self._as.publish_feedback(self._feedback)

            if dist_from_goal < 0.1:
                print('done')
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                self._cmd_vel_pub.publish(speed)
                self._result.done = True
                self._as.set_succeeded(self._result)
                break

            if abs(angle_to_goal - yaw) > 0.2:
                speed.linear.x = 0.0
            else:
                speed.linear.x = 0.4

            rospy.logdebug(f'angle_to_goal: {angle_to_goal}')
            rospy.logdebug(f'yaw: {yaw}\n')

            if angle_to_goal - yaw > 0.01:
                speed.angular.z = 0.3
            elif angle_to_goal - yaw < -0.01:
                speed.angular.z = -0.3

            self._cmd_vel_pub.publish(speed)
            r.sleep()

    @staticmethod
    def get_world_from_grid_coordinate(grid: Point):
        x_pos = grid.x + 0.5
        y_pos = grid.y + 0.5
        return Point(x=x_pos, y=y_pos)


if __name__ == '__main__':
    try:
        rospy.init_node('go_to_grid_cell_server')
        server = GoToGridCell()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
