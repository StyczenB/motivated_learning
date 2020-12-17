#!/usr/bin/env python3
import rospy

from gazebo_msgs.msg import LinkStates
from robot_msgs.msg import PainsMsg

from robot.agent import Agent
from robot.pains import Pains
from robot.robot_pose_manager import RobotPoseManager
from environment.chargers_manager import ChargersManager

PAIN_NAMES = {name: idx - 1 for idx, name in enumerate(PainsMsg.__slots__) if name != 'header'}
PAIN_IDX = {idx: name for name, idx in PAIN_NAMES.items()}
# print(PAIN_NAMES)


def check_simulation_state():
    # if this function returns gazebo is running
    rospy.logdebug('check simulation state')
    link_states = rospy.wait_for_message('/gazebo/link_states', LinkStates)


def get_dominant_pain(pains: PainsMsg) -> (int, str, float):
    pains_values = [pains.__getattribute__(pain_name) for pain_name in PAIN_NAMES.keys()]
    dominant_pain_val = max(pains_values)
    dominant_pain_idx = pains_values.index(dominant_pain_val)
    return {'idx': dominant_pain_idx, 'name': PAIN_IDX[dominant_pain_idx], 'val': dominant_pain_val}


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
            # print(current_pains)
            dominant_pain = get_dominant_pain(current_pains)
            print(dominant_pain)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    # check current position - pains, get state
    #
