#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from robot_msgs.msg import GridCoord
import tf
import math
import copy


class UpdateAgentGridCoord:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, callback=self.odom_cb, queue_size=1)
        self.grid_cell_coord_pub = rospy.Publisher('grid_coord', GridCoord, queue_size=1)

    def odom_cb(self, data: Odometry):
        grid_coord = GridCoord()
        grid_coord.header = data.header
        grid_coord.x = int(data.pose.pose.position.x) 
        grid_coord.y = int(data.pose.pose.position.y) 
        self.grid_cell_coord_pub.publish(grid_coord)


if __name__ == '__main__':
    try:
        rospy.init_node('update_agent_grid_coord', anonymous=True)
        grid_coord_updater = UpdateAgentGridCoord()
        rospy.loginfo('Started node with agent grid coord updater.')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


