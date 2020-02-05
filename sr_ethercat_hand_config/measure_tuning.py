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


class MeasureTuning(object):
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

    def __init__(self, output_file, hand_commander_group="right_hand"):
        rospy.loginfo("Starting hand commander for group %s." % hand_commander_group)
        self._hand_commander = SrHandCommander(name=hand_commander_group)
        rospy.loginfo("Opening %s for output.\n" % output_file)
        self._out_file = open(output_file, 'w')

        self._out_file.write("joint, time, integral, stddev, diffs\n")
        self._errors = []

    def _error_cb(self, msg):
        self._errors.append(msg.error)

    def do_run(self, time, joint, pause_time=0.2):
        topic = "/sh_%s_position_controller/state" % joint.lower()

        print "length " + str(len(self._errors))

        self._errors = []

        print "length now " + str(len(self._errors))

        subscriber = rospy.Subscriber(topic, JointControllerState, self._error_cb)

        rospy.loginfo("Subscribed to " + topic)

        flex = self.flex
        extend = self.extend
        hand_commander = self._hand_commander

        if not rospy.is_shutdown():
            hand_commander.move_to_joint_value_target_unsafe(flex[joint], time, True, angle_degrees=True)
            rospy.sleep(pause_time)
        if not rospy.is_shutdown():
            hand_commander.move_to_joint_value_target_unsafe(extend[joint], time, True, angle_degrees=True)

        subscriber.unregister()
        errors = self._errors

        rospy.sleep(1)

        diff_list = [ abs(errors[n] - errors[n+1]) for n in range(len(errors) -1) ]

        integral = (str(np.sum(np.absolute(errors))/len(errors)))
        stdev = (str(np.std(errors)))
        diffs = (str(np.sum(diff_list)/len(errors)))

        rospy.loginfo("Test over " + str(time) + "s finished for " + joint +".")
        rospy.loginfo("integrated error: " + integral)
        rospy.loginfo("stdev of errors: " + stdev)
        rospy.loginfo("diffs of errors: " + diffs + "\n\n")

        rospy.sleep(1)
        output = ", ".join(
            [joint, str(time),  integral, stdev, diffs]
        )

        self._out_file.write(output+"\n")


output_file = argv[1]
print "output file - " + output_file

measure = MeasureTuning(output_file);


joints = ["rh_FFJ0" ,"rh_FFJ3", "rh_FFJ4",
          "rh_MFJ0" ,"rh_MFJ3", #"rh_MFJ4",
          "rh_RFJ0" ,"rh_RFJ3", #"rh_RFJ4",
          "rh_LFJ0" ,"rh_LFJ3", "rh_LFJ5", #"rh_LFJ4",
          "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5"]

#joints = ["rh_FFJ0", "rh_LFJ0" ]

ts = [0.25 , 0.5, 1.0, 2.0, 10.0]

rospy.sleep(1)

for joint in joints:
    for t in ts:
        if not rospy.is_shutdown():
            print "about to run"
            measure.do_run(t, joint)
            print "did run"
