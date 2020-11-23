#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import tf
import math
import copy

class Robot:
    IDLE = 0
    MOVING = 1
    CHARGING = 2

    def __init__(self):
        self.battery_charge = 100
        self.idle_discharge_coeff = 0.99
        self.moving_discharge_coeff = 1.0
        

    def step(self):
        self.battery_charge *= 0.99
        


def get_robot_pose():
    odom = rospy.wait_for_message('/odom', Odometry)
    pos = odom.pose.pose.position.x, odom.pose.pose.position.y
    quat = odom.pose.pose.orientation
    _, _, yaw = tf.transformations.euler_from_quaternion(
        [quat.x, quat.y, quat.z, quat.w])
    return pos[0], pos[1], yaw


# if __name__ == '__main__':
#     try:
#         rospy.init_node('gazebo_model_state', anonymous=True)
#         # prev_pos_x, prev_pos_y, prev_yaw = 0, 0, 0
#         prev_pose = 0, 0, 0
#         rate = rospy.Rate(100)
#         while not rospy.is_shutdown():
#             curr_pose = get_robot_pose()
#             print(f'prev_pose: {prev_pose}')
#             print(f'curr_pose: {curr_pose}')
#             diff = curr_pose[0] - prev_pose[0], curr_pose[1] - prev_pose[1], curr_pose[2] - prev_pose[2]
#             pose_changed = not (math.isclose(diff[0], 0, abs_tol=1e-4) or math.isclose(diff[1], 0, abs_tol=1e-4) or math.isclose(diff[2], 0, abs_tol=1e-4))
#             print(f'diff pose: {pose_changed}')

#             prev_pose = copy.copy(curr_pose)
#             print('---')

#             rate.sleep()
#     except rospy.ROSInterruptException:
#         pass

if __name__ == '__main__':
    try:
        rospy.init_node('gazebo_model_state', anonymous=True)
        listener = tf.TransformListener()
        rate = rospy.Rate(100)

        # listener.waitForTransform("/base_footprint", "/odom", rospy.Time(), rospy.Duration(5))
        while not rospy.is_shutdown():
            try:
                trans, rot = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            print(trans)
            print(rot)
            print('---')
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
