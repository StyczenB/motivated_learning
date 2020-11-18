#! /usr/bin/env python3
import sys
import rospy
import actionlib
from robot_msgs.msg import GoToGridCellAction, GoToGridCellGoal, GoToGridCellFeedback, GoToGridCellResult
from geometry_msgs.msg import Point


def feedback_cb(feedback):
    rospy.loginfo(f'\nFeedback: {feedback.dist_to_goal} m')


if __name__ == '__main__':
    try:
        rospy.init_node('go_to_grid_cell_client')
        if (len(sys.argv) != 3):
            rospy.logwarn('Not enough parameters.')
            exit(0)

        client = actionlib.SimpleActionClient('go_to_grid_cell', GoToGridCellAction)
        client.wait_for_server()
        rospy.loginfo('Action server avaiable')

        goal = GoToGridCellGoal()
        goal.grid_coordinates = Point(x=float(sys.argv[1]), y=float(sys.argv[2]))
        client.send_goal(goal)
        # client.send_goal(goal, feedback_cb=feedback_cb)

        client.wait_for_result()
        print('[Result] State: %d' % (client.get_state()))

    except rospy.ROSInterruptException:
        pass
