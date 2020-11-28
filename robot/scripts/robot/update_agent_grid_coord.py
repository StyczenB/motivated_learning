#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from robot_msgs.msg import FieldMsg
import math


class UpdateAgentGridCoord:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, callback=self.odom_cb, queue_size=1)
        self.grid_cell_coord_pub = rospy.Publisher('grid_coord', Point, queue_size=1)

    def odom_cb(self, data: Odometry):
        grid_coord = Point()
        x, y = data.pose.pose.position.x, data.pose.pose.position.y
        grid_coord.x = math.floor(x + 0.5) 
        grid_coord.y = math.floor(y + 0.5) 
        self.grid_cell_coord_pub.publish(grid_coord)


if __name__ == '__main__':
    try:
        rospy.init_node('update_agent_grid_coord', anonymous=True)
        grid_coord_updater = UpdateAgentGridCoord()
        rospy.loginfo('Started node with agent grid coord updater.')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


