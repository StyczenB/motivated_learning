#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.msg import LinkStates
from robot.agent import Agent
from robot.robot_pose_manager import RobotPoseManager
from environment.chargers_manager import ChargersManager


def check_simulation_state():
    # if this function returns gazebo is running
    link_states = rospy.wait_for_message('/gazebo/link_states', LinkStates)


if __name__ == '__main__':
    try:
        rospy.init_node('robot', anonymous=True)
        agent = Agent()
        chargers_manager = ChargersManager()
        robot_pose_manager = RobotPoseManager()

        rospy.loginfo('Started node with agent state updaters.')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            check_simulation_state()
            agent.step()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    # check current position - pains, get state
    #
