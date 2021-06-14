from typing import Dict, List
import matplotlib.pyplot as plt
from collections import defaultdict
import rospy
from robot_msgs.msg import PainsMsg


class LoggingPainsData:
    def __init__(self):
        self._data: Dict[str, List[float]] = defaultdict(list)
        self._pains_sub = rospy.Subscriber('pains', PainsMsg, self._pains_callback)
    
    def __del__(self):
        print('dumping to file')
        print(self._data)
        plt.plot('header', 'low_battery_level', data=self._data)
        plt.show()

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

