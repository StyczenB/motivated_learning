#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import tf
import math
from environment.create_map import GlobalMapManager
from environment.update_chargers import ChargersManager


if __name__ == '__main__':
    try:
        rospy.init_node('main_program', anonymous=True)

        # global_map_manager = GlobalMapManager()
        # chargers_manager = ChargersManager()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.wait_for_service("/gazebo/get_model_state")

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
