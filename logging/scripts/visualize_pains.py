#!/usr/bin/env python3
import os
import json
import matplotlib.pyplot as plt
import rospy
from robot_msgs.msg import PainsMsg


if __name__ == '__main__':
    try:
        rospy.init_node('visualization', anonymous=True)
        
        path = rospy.get_param('data_path')
        with open(path, 'r') as f:
            data = json.load(f)
        timestamps = data.pop('header')

        time_offset = timestamps[0]
        t = [ts - time_offset for ts in timestamps]
        print(t)
        nr_pains = len(data.keys())
        print(f'Logged number of pains: {nr_pains}')

        fig, axs = plt.subplots(nr_pains)
        for i, (pain_name, pain_values) in enumerate(data.items()):
            axs[i].plot(t, pain_values)
            axs[i].set_title(pain_name)
        fig.tight_layout()
        plt.show()

    except Exception as e:
        print(e)

