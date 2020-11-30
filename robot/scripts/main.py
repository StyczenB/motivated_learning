#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.msg import LinkStates
from robot.agent import Agent


def check_simulation_state():
    # if this function returns gazebo is running
    link_states = rospy.wait_for_message('/gazebo/link_states', LinkStates)


if __name__ == '__main__':
    try:
        rospy.init_node('robot', anonymous=True)
        agent = Agent()
        rospy.loginfo('Started node with agent state updaters.')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            check_simulation_state()
            agent.step()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass