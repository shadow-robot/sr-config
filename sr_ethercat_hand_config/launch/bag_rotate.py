#!/usr/bin/env python
import os
import time
import rospy


def remover(desired_bag_number, path):
    while not rospy.is_shutdown():
        bag_files = [f for f in os.listdir(path) if f.endswith('.bag')]

        sorted_bag_files = sorted(bag_files, key=lambda x: os.path.getctime(os.path.join(path, x)))

        while len(sorted_bag_files) > desired_bag_number:
            os.remove(os.path.join(path, sorted_bag_files[0]))
            sorted_bag_files.pop(0)

        time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('bag_rotate', anonymous=True)
    desired_bag_number = rospy.get_param('~bag_files_num')
    path = rospy.get_param('~bag_files_path')
    remover(desired_bag_number, path)
