#!/usr/bin/env python

# This script can be used to gently squeeze a bottle in a motion which oscillates 3 times between a 'hold' position
# and a tight squeeze. The sequence begins and ends with an open hand.

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("bottle_squeeze", anonymous=True)

hand_commander = SrHandCommander()

# Specify joint goals for hand
hand_joints_goal_1 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 35.0, 'rh_FFJ3': 30.0, 'rh_FFJ4': -15.0,
                      'rh_MFJ1': 0.0, 'rh_MFJ2': 35.0, 'rh_MFJ3': 30.0, 'rh_MFJ4': -10.0,
                      'rh_RFJ1': 0.0, 'rh_RFJ2': 35.0, 'rh_RFJ3': 30.0, 'rh_RFJ4': -10.0,
                      'rh_LFJ1': 0.0, 'rh_LFJ2': 35.0, 'rh_LFJ3': 30.0, 'rh_LFJ4': -15.0, 'rh_LFJ5': 0.0,	
                      'rh_THJ1': 10.0, 'rh_THJ2': 10.0, 'rh_THJ3': 10.0, 'rh_THJ4': 70.0, 'rh_THJ5': -8.0,
                      'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

hand_joints_goal_2 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 40.0, 'rh_FFJ3': 40.0, 'rh_FFJ4': -15.0,
                      'rh_MFJ1': 0.0, 'rh_MFJ2': 40.0, 'rh_MFJ3': 40.0, 'rh_MFJ4': -10.0,
                      'rh_RFJ1': 0.0, 'rh_RFJ2': 40.0, 'rh_RFJ3': 40.0, 'rh_RFJ4': -10.0,
                      'rh_LFJ1': 0.0, 'rh_LFJ2': 40.0, 'rh_LFJ3': 40.0, 'rh_LFJ4': -15.0, 'rh_LFJ5': 0.0,	
                      'rh_THJ1': 30.0, 'rh_THJ2': 15.0, 'rh_THJ3': 10.0, 'rh_THJ4': 70.0, 'rh_THJ5': 10.0,
                      'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

hand_joints_goal_3 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 40.0, 'rh_FFJ3': 50.0, 'rh_FFJ4': -15.0,
                      'rh_MFJ1': 0.0, 'rh_MFJ2': 40.0, 'rh_MFJ3': 50.0, 'rh_MFJ4': -10.0,
                      'rh_RFJ1': 0.0, 'rh_RFJ2': 40.0, 'rh_RFJ3': 50.0, 'rh_RFJ4': -10.0,
                      'rh_LFJ1': 0.0, 'rh_LFJ2': 40.0, 'rh_LFJ3': 50.0, 'rh_LFJ4': -15.0, 'rh_LFJ5': 0.0,	
                      'rh_THJ1': 30.0, 'rh_THJ2': 20.0, 'rh_THJ3': 10.0, 'rh_THJ4': 70.0, 'rh_THJ5': 10.0,
                      'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

# Move through each goal
# joint states are sent to the commanders, with a time for execution and a flag as to whether
# or not the commander should wait for the command to complete before moving to the next command.

# Move hand
hand_commander.move_to_named_target("open")
rospy.sleep(2)
hand_commander.move_to_joint_value_target_unsafe(hand_joints_goal_1, 2.0, False, angle_degrees=True)
rospy.sleep(4)

for x in range(0, 3):
    joint_goals = hand_joints_goal_2
    rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
    hand_commander.move_to_joint_value_target_unsafe(joint_goals, 1.0, False, angle_degrees=True)
    rospy.sleep(1.0)
    joint_goals = hand_joints_goal_3
    rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
    hand_commander.move_to_joint_value_target_unsafe(joint_goals, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)

rospy.sleep(2)
hand_commander.move_to_joint_value_target_unsafe(hand_joints_goal_1, 2.0, False, angle_degrees=True)
rospy.sleep(2)
hand_commander.move_to_named_target("open")
