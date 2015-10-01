#!/usr/bin/env python

# Script to move the right hand into store position.

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("store_right_hand", anonymous=True)

hand_commander = SrHandCommander(name="rh_shadow_hand", prefix="rh")

open_hand = {'rh_FFJ1': -0.3704364055719059, 'rh_FFJ2': 2.9104152550576012, 'rh_FFJ3': -0.9724461175055554,
             'rh_FFJ4': -21.32891685466406, 'rh_THJ4': 1.0409733252891025, 'rh_THJ5': -0.6686710349194556,
             'rh_THJ1': 1.14620021934065, 'rh_THJ2': 0.5758761765422591, 'rh_RFJ4': -19.17625073560172,
             'rh_RFJ1': 0.7935107801372315, 'rh_RFJ2': 1.1797543398771682, 'rh_RFJ3': -2.085509874580722}

pack_hand = {'rh_FFJ1': 69.63013127734945, 'rh_FFJ2': 96.51810584958217, 'rh_FFJ3': 86.70044944010863,
             'rh_FFJ4': 0.4041137911705101, 'rh_THJ4': 1.2094462329571531, 'rh_THJ5': -0.6124558839601048,
             'rh_THJ1': 1.1248750890729802, 'rh_THJ2': 0.6961920903067368, 'rh_RFJ4': -1.4931514966098631,
             'rh_RFJ1': 44.47422004181642, 'rh_RFJ2': 89.6428995699589, 'rh_RFJ3': 86.44557820905897}



# Move hand to open position
joint_states = open_hand
rospy.loginfo("Moving hand to open position")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, True, angle_degrees=True)
rospy.sleep(2)

# Move hand to closed position
joint_states = pack_hand
rospy.loginfo("Moving hand to pack position")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, True, angle_degrees=True)
rospy.sleep(2)
