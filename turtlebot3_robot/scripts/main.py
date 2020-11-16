#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.srv import GetModelState
import tf2_py

if __name__ == '__main__':
    try:
        rospy.init_node('gazebo_model_state', anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.wait_for_service("/gazebo/get_model_state")
            try:
                get_model_state = rospy.ServiceProxy(
                    "/gazebo/get_model_state", GetModelState)
                res = get_model_state("turtlebot3_waffle", '')
                pos = (res.pose.position.x, res.pose.position.y)
                quat = res.pose.orientation
                yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
                # print 'x:', pos[0]
                # print 'y:', pos[1]
                # print 'yaw:', yaw, '\n'
            except rospy.ServiceException as e:
                print("Error...")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
