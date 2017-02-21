#!/usr/bin/env python

# Script to move the left hand into store position.

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("store_right_hand", anonymous=True)

hand_commander = SrHandCommander(name="right_hand")

open_hand = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': -8,
             'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': -3,
             'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': -3,
             'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

pack_hand_1 = {'rh_FFJ1': 90, 'rh_FFJ2': 90, 'rh_FFJ3': 90, 'rh_FFJ4': 4,
               'rh_MFJ1': 90, 'rh_MFJ2': 90, 'rh_MFJ3': 90, 'rh_MFJ4': 1,
               'rh_RFJ1': 90, 'rh_RFJ2': 90, 'rh_RFJ3': 90, 'rh_RFJ4': 1}

pack_hand_2 = {'rh_THJ4': 67}

pack_hand_3 = {'rh_THJ1': 50, 'rh_THJ2': 22, 'rh_THJ5': 18}

open_thumb = {'rh_THJ1': 0, 'rh_THJ2': 0, 'rh_THJ5': 0}

# Move hand to open position

joint_states = open_thumb
rospy.loginfo("Moving hand to open position")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, False, angle_degrees=True)
rospy.sleep(2.0)

joint_states = open_hand
rospy.loginfo("Moving hand to open position")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, False, angle_degrees=True)
rospy.sleep(3)

# Move hand to closed position
joint_states = pack_hand_1
rospy.loginfo("Moving hand to pack position")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 5.0, False, angle_degrees=True)
rospy.sleep(5)

joint_states = pack_hand_2
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, False, angle_degrees=True)
rospy.sleep(2)

joint_states = pack_hand_3
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, False, angle_degrees=True)
rospy.sleep(2)



