#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import tf
import math


def get_grid_from_world_coordinate(world_x, world_y):
    grid_x = math.floor(world_x)
    grid_y = math.floor(world_y)
    return Point(x=grid_x, y=grid_y)


def get_world_from_grid_coordinate(grid: Point):
    x_pos = grid.x + 0.5
    y_pos = grid.y + 0.5
    return Point(x=x_pos, y=y_pos)


if __name__ == '__main__':
    try:
        rospy.init_node('gazebo_model_state', anonymous=True)
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        get_model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.wait_for_service("/gazebo/get_model_state")
            try:
                # res = get_model_state("turtlebot3_waffle", '')
                # pos1 = res.pose.position.x, res.pose.position.y
                # quat = res.pose.orientation
                # _, _, yaw1 = tf.transformations.euler_from_quaternion(
                #     [quat.x, quat.y, quat.z, quat.w])

                odom = rospy.wait_for_message('/odom', Odometry)
                pos = odom.pose.pose.position.x, odom.pose.pose.position.y  
                quat = odom.pose.pose.orientation
                _, _, yaw = tf.transformations.euler_from_quaternion(
                    [quat.x, quat.y, quat.z, quat.w])
                print(f'Pos from odom: {pos}')
                # print(f'Pos from GetM: {pos1}')
                print(f'Yaw from odom: {yaw}')
                # print(f'Yaw from GetM: {yaw1}\n')

                inc_x = 1 
                inc_y = -1
                angle_to_goal = math.atan2(inc_y, inc_x)
                print(f'angle_to_goal: {angle_to_goal}')

                print()
            except rospy.ServiceException as e:
                print("Error...")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
