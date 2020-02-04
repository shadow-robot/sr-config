#!/usr/bin/env python

# Copyright 2020 Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import rospy
from sys import argv
from copy import deepcopy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from control_msgs.msg import JointControllerState
import numpy as np

rospy.init_node("move_finger", anonymous=True)

flex = {
    "rh_FFJ0" : {"rh_FFJ1": 90, "rh_FFJ2": 90},
    "rh_MFJ0" : {"rh_MFJ1": 90, "rh_MFJ2": 90},
    "rh_RFJ0" : {"rh_RFJ1": 90, "rh_RFJ2": 90},
    "rh_LFJ0" : {"rh_LFJ1": 90, "rh_LFJ2": 90},
    "rh_FFJ3" : {"rh_FFJ3": 80},
    "rh_MFJ3" : {"rh_MFJ3": 80},
    "rh_RFJ3" : {"rh_RFJ3": 80},
    "rh_LFJ3" : {"rh_LFJ3": 80},
    "rh_FFJ4" : {"rh_FFJ4": 15},
    "rh_MFJ4" : {"rh_MFJ4": 15},
    "rh_RFJ4" : {"rh_RFJ4": -15},
    "rh_LFJ4" : {"rh_LFJ4": -15},
    "rh_LFJ5" : {"rh_LFJ5": 45},
    "rh_THJ1" : {"rh_THJ1": 90},
    "rh_THJ2" : {"rh_THJ2": 15},
    "rh_THJ3" : {"rh_THJ3": 10},
    "rh_THJ4" : {"rh_THJ4": 45},
    "rh_THJ5" : {"rh_THJ5": 60}
}

extend = {
    "rh_FFJ0" : {"rh_FFJ1": 0, "rh_FFJ2": 0},
    "rh_MFJ0" : {"rh_MFJ1": 0, "rh_MFJ2": 0},
    "rh_RFJ0" : {"rh_RFJ1": 0, "rh_RFJ2": 0},
    "rh_LFJ0" : {"rh_LFJ1": 0, "rh_LFJ2": 0},
    "rh_FFJ3" : {"rh_FFJ3": 0},
    "rh_MFJ3" : {"rh_MFJ3": 0},
    "rh_RFJ3" : {"rh_RFJ3": 0},
    "rh_LFJ3" : {"rh_LFJ3": 0},
    "rh_FFJ4" : {"rh_FFJ4": -15},
    "rh_MFJ4" : {"rh_MFJ4": -15},
    "rh_RFJ4" : {"rh_RFJ4": 15},
    "rh_LFJ4" : {"rh_LFJ4": 15},
    "rh_LFJ5" : {"rh_LFJ5": 0},
    "rh_THJ1" : {"rh_THJ1": -10},
    "rh_THJ2" : {"rh_THJ2": -15},
    "rh_THJ3" : {"rh_THJ3": -10},
    "rh_THJ4" : {"rh_THJ4": 0},
    "rh_THJ5" : {"rh_THJ5": 0},
}


output_file = argv[1]
print "output file - " + output_file

mid = {}

for main_joint in flex.keys():
    mid[main_joint] = { n : (flex[main_joint][n] + extend[main_joint][n])/2 for n in flex[main_joint].keys() }


hand_commander = SrHandCommander(name="right_hand")

# time = float(argv[1])
# joint = argv[2]

pause_time = 0.2


def do_run(time, pause_time, joint):
    if not rospy.is_shutdown():
        hand_commander.move_to_joint_value_target_unsafe(flex[joint], time, True, angle_degrees=True)
        rospy.sleep(pause_time)
    if not rospy.is_shutdown():
        hand_commander.move_to_joint_value_target_unsafe(extend[joint], time, True, angle_degrees=True)

total_error = 0.0

errors = []

def error_cb(msg):
    errors.append(msg.error)


joints = ["rh_FFJ0" ,"rh_FFJ3", "rh_FFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5"]


all_integrate = []
all_stdev = []
all_diffs = []
ts = [.25 , 0.5, 1.0, 2.0, 10.0]

for joint in joints:
    topic = "/sh_%s_position_controller/state" % joint.lower()
    subscriber = rospy.Subscriber(topic, JointControllerState, error_cb)
    rospy.loginfo("Subscribed to " + topic)
    integrate = []
    stdev = []
    diffs = []

    for t in ts:
        if rospy.is_shutdown():
            exit()
        errors = []
        do_run(t, 0.2, joint)
        rospy.sleep(1)

        diff = [ abs(errors[n] - errors[n+1]) for n in range(len(errors) -1) ]

        integrate.append(str(np.sum(np.absolute(errors))/len(errors)))
        stdev.append(str(np.std(errors)))
        diffs.append(str(np.sum(diff)/len(errors)))

        rospy.loginfo("Test over " + str(t) + "s finished for " + joint +".")
        rospy.loginfo("integrated error: " + str(np.mean(np.absolute(errors))))
        rospy.loginfo("stdev of errors: " + str(np.std(np.absolute(errors))))
        rospy.loginfo("diffs of errors: " + str(np.mean(diff)))

    all_integrate.append(integrate)
    all_stdev.append(stdev)
    all_diffs.append(diffs)

if rospy.is_shutdown():
    exit()

with open(output_file, 'w') as out:
    header = "joint, time, integral, stddev, diffs"
    print header
    out.write(header+"\n")
    for j, joint in enumerate(joints):
        for n, t in enumerate(ts):
            output = ", ".join(
                [joint, str(t),  all_integrate[j][n], all_stdev[j][n], all_diffs[j][n]]
            )
            print(output)
            out.write(output+"\n")
