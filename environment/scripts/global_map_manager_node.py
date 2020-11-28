#!/usr/bin/env python3
import rospy
from environment.global_map_manager import GlobalMapManager
import time
import sys
import os


if __name__ == '__main__':
    try:
        rospy.init_node('create_map')
        global_map_manager = GlobalMapManager()

        start = time.time()
        global_map_manager.create_map()
        stop = time.time()
        rospy.loginfo(f'Creating map execution time: {stop - start} sec.')

        rospy.loginfo('Map loaded')
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Error while creating map with message: {e}')
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(exc_type, fname, exc_tb.tb_lineno)
