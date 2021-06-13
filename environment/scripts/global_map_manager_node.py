#!/usr/bin/env python3
import time
import sys
import os
import argparse
import rospy
from environment.global_map_manager import GlobalMapManager


if __name__ == '__main__':
    try:
        rospy.init_node('global_map_manager')
        global_map_manager = GlobalMapManager()
        
        # Get parameters from parameter server in this node's namespace
        random_map = rospy.get_param('random_map')
        x_size = rospy.get_param('x_size')
        y_size = rospy.get_param('y_size')
        
        # Create previously defined map or generate random
        start = time.time()
        global_map_manager.create_map(random_map, x_size, y_size)
        stop = time.time()
        rospy.loginfo(f'Creating map execution time: {stop - start} sec.')

        rospy.loginfo('Map loaded')
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Error while creating map with message: {e}')
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(exc_type, fname, exc_tb.tb_lineno)

