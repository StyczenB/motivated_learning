#! /usr/bin/env python3
import rospy
from robot.movement_manager import MovementManager

if __name__ == '__main__':
    try:
        rospy.init_node('movement_manager', log_level=rospy.INFO)
        movement_manager = MovementManager()
        rospy.loginfo('Movement manager started...')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
