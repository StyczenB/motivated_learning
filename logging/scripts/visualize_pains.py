#!/usr/bin/env python3
from datetime import datetime
import os
import json
import matplotlib.pyplot as plt
import rospy
from robot_msgs.msg import PainsMsg


if __name__ == '__main__':
    try:
        rospy.init_node('visualization', anonymous=True)
        
        dir_path = rospy.get_param('data_path')
        files = os.listdir(dir_path)
        files.sort()
        if len(files) > 0:
            print('Logs to display')
            for i, file_name in enumerate(files):
                print(f'  [{i}] {file_name}')
            file_index = int(input('Select log to display: '))
            file_to_load = files[file_index]
            path = os.path.join(dir_path, file_to_load)
        else:
            print('No logs to display')
            exit(0)
        
        with open(path, 'r') as f:
            data = json.load(f)
        timestamps = data.pop('header')

        time_offset = timestamps[0]
        t = [ts - time_offset for ts in timestamps]
        nr_pains = len(data.keys())
        print(f'Logged number of pains: {nr_pains}')

        fig, axs = plt.subplots(nrows=nr_pains, ncols=1, sharex=True, constrained_layout=True)
#        dt = datetime.strptime(file_to_load.split('.')[0], '%Y%m%d%H%M%S')
#        fig.suptitle(f'{dt.ctime()}')
        for i, (pain_name, pain_values) in enumerate(data.items()):
            axs[i].plot(t, pain_values)
            axs[i].set_title(pain_name.replace('_', ' '))
            axs[i].grid()
        plt.xlabel('time [s]')
        plt.show()

    except Exception as e:
        print(e)

