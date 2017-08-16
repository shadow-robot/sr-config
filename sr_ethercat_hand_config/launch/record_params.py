#!/usr/bin/env python

import rospy
import datetime

rospy.init_node("param_dump", anonymous=True)

run_time = datetime.datetime.fromtimestamp(rospy.get_rostime().secs)

name_string = "run_params_%04d-%02d-%02d-%02d-%02d-%02d" % (
    run_time.year, run_time.month, run_time.day, run_time.hour, run_time.minute, run_time.second)

rospy.sleep(10)

param_tree = rospy.get_param("/")

output = open(rospy.get_param("~log_directory", ".") + "/" + name_string, 'w')

output.write(str(param_tree))
