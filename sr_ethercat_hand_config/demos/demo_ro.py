#!/usr/bin/env python3

# Script to move the left hand into open position.

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("open_right_hand", anonymous=True)

hand_commander = SrHandCommander(name="right_hand")

open_hand = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
             'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
             'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
             'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
             'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0,
             'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

# Move hand to open position
joint_states = open_hand
rospy.loginfo("Moving hand to open position")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, False)
rospy.sleep(2)
