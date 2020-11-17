#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, Point
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

        res = get_model_state("turtlebot3_waffle", '')
        pos = (res.pose.position.x, res.pose.position.y)
        starting_grid_coord = get_grid_from_world_coordinate(*pos)

        goal_grid = Point(x=starting_grid_coord.x-1, y=starting_grid_coord.y+1)
        goal_world = get_world_from_grid_coordinate(goal_grid)
        print(f'Goal: {goal_world}')

        speed = Twist()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.wait_for_service("/gazebo/get_model_state")
            try:
                res = get_model_state("turtlebot3_waffle", '')
                pos = (res.pose.position.x, res.pose.position.y)
                quat = res.pose.orientation
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                    [quat.x, quat.y, quat.z, quat.w])
                # grid_coord = get_grid_from_world_coordinate(*pos)
                # print(f'Goal: {goal_world}')
                # print(f'Current: {pos}')

                # set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
                # goal_model_state = ModelState()
                # goal_model_state.model_name = "turtlebot3_waffle"
                # goal_model_state.pose.position.x = goal_world.x
                # goal_model_state.pose.position.y = goal_world.y
                # set_model_state(goal_model_state)
                # break

                inc_x = goal_world.x - pos[0]
                inc_y = goal_world.y - pos[1]
                angle_to_goal = math.atan2(inc_y, inc_x)
                
                dist_from_goal = math.sqrt(inc_x**2 + inc_y**2)
                print(f'dist_from_goal: {dist_from_goal}')
                if dist_from_goal < 0.1:
                    print('done')
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    cmd_vel_pub.publish(speed)
                    break

                if abs(angle_to_goal - yaw) > 0.25:
                    speed.linear.x = 0.0
                else:
                    speed.linear.x = 0.3

                print(angle_to_goal)
                print(yaw)
                if angle_to_goal - yaw > 0.1:
                    speed.angular.z = 0.3
                elif angle_to_goal - yaw < -0.1:
                    speed.angular.z = -0.3

                cmd_vel_pub.publish(speed)

            except rospy.ServiceException as e:
                print("Error...")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
