#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


class GridCell:
    def __init__(self):
        pass

    def do_stuff(self):
        print('Do stuff')


if __name__ == '__main__':
    try:
        rospy.init_node('grid_cell', anonymous=True)
        grid_cell = GridCell()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            grid_cell.do_stuff()
            # pub.publish("hello_str")
            rate.sleep()
    except rospy.ROSInterruptException as e:
        print(e)

