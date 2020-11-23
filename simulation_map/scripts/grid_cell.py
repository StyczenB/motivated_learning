#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point


class GridCell:
    def __init__(self, neuron=None):
        self.coordinates = Point()
        self.nr_visists = 0
        self.neuron = neuron


if __name__ == '__main__':
    try:
        rospy.init_node('grid_cell', anonymous=True)
        grid_cell = GridCell()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # pub.publish("hello_str")
            rate.sleep()
    except rospy.ROSInterruptException as e:
        print(e)
