from datetime import datetime
import os
import time
import atexit
import json
from typing import Dict, List
from collections import defaultdict
import rospy
from robot_msgs.msg import PainsMsg


class LoggingPainsData:
    def __init__(self):
        self._data: Dict[str, List[float]] = defaultdict(list)
        self._pains_sub = rospy.Subscriber('pains', PainsMsg, self._pains_callback)
        atexit.register(self._save_to_file)

    def _save_to_file(self):
        dir_path = rospy.get_param('data_path')
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        date_time = datetime.now().strftime('%Y%m%d%H%M%S')
        path = os.path.join(dir_path, f'{date_time}.json')
        with open(path, 'w') as f:
            json.dump(self._data, f)

    def _pains_callback(self, pains_data: PainsMsg):
        self._data['header'].append(pains_data.header.stamp.secs + pains_data.header.stamp.nsecs / 1e9)
        pain_names = [pain_name for pain_name in pains_data.__slots__ if pain_name != 'header']
        for pain_name in pain_names:
            self._data[pain_name].append(getattr(pains_data, pain_name))


if __name__ == '__main__':
    try:
        rospy.init_node('logging', anonymous=True)
        rospy.loginfo('Starting logging node')

        pains_logger = LoggingPainsData()

        rospy.spin()
    except Exception as e:
        print(e)

