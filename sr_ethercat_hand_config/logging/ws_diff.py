#!/usr/bin/env python

import os
import rospy
import datetime
import subprocess
import gzip

def recursive_diff(path):
    if not os.path.isdir(path):
        return ""
    current_directory = os.getcwd()
    os.chdir(path)
    output = ""
    if os.path.isdir("./.git"):
        output = "\n\n------------------" + path + "------------------\n\n"
        output += subprocess.check_output(['git', 'show'])
        output += subprocess.check_output(['git', 'status'])
        output += subprocess.check_output(['git', 'diff'])

    for subdir in filter (os.path.isdir, os.listdir(".")):
        output += recursive_diff(subdir)

    os.chdir(current_directory)
    return output

rospy.init_node("ws_diff")

package_path = os.environ['ROS_PACKAGE_PATH']
package_dirs = filter (lambda x: ("/opt/ros" not in x), package_path.split(":"))

run_time = datetime.datetime.fromtimestamp(rospy.get_rostime().secs)
name_string = "ws_diff_%04d-%02d-%02d-%02d-%02d-%02d" % (
    run_time.year, run_time.month, run_time.day, run_time.hour, run_time.minute, run_time.second)

output_dir = rospy.get_param("~log_directory", ".")

output_path = "%s/wsdiff_%s.gz" % (output_dir, name_string)

output = package_path + "\n"

for directory in package_dirs:
    output = output + recursive_diff(directory)
with gzip.open(output_path,'wb') as f:
    f.write(output)
