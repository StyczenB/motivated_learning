#!/usr/bin/env python3
import rospy
# import sys
from gazebo_msgs.msg import LinkStates
# from robot_msgs.msg import PainsMsg
from robot.agent import Agent
from robot.pains import Pains
from robot.robot_pose_manager import RobotPoseManager
from environment.chargers_manager import ChargersManager


def check_simulation_state():
    # if this function returns gazebo is running
    rospy.logdebug('check simulation state')
    link_states = rospy.wait_for_message('/gazebo/link_states', LinkStates)


# def get_dominant_pain(pains: PainsMsg) -> int:
#     PainsMsg.__slots__


if __name__ == '__main__':
    try:
        rospy.init_node('robot', anonymous=True, log_level=rospy.INFO)

        chargers_manager = ChargersManager()
        robot_pose_manager = RobotPoseManager()
        
        agent = Agent()
        pains = Pains()

        rospy.loginfo('Started node with agent state updaters.')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            check_simulation_state()  # This function only freeze running of loop when Gazebo simulation is paused
            agent.step()
            current_pains = pains.step()
            print(current_pains)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    # check current position - pains, get state
    #
