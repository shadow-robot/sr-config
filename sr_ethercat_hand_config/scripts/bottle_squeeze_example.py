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

hand_joints_goal_10 = {'rh_FFJ1': 0.3438351595450134, 'rh_FFJ2': 1.0716025788727896, 'rh_FFJ3': 0.44917149949781676,
                      'rh_FFJ4': -0.25129918457338096, 'rh_THJ4': 1.222646154278984, 'rh_THJ5': 0.20251875325330035,
                      'rh_THJ1': 0.8555994987350249, 'rh_THJ2': 0.012552554413840782, 'rh_THJ3': 0.217009535407161,
                      'rh_LFJ2': 1.2457011376417755, 'rh_LFJ3': 0.39648003569506673, 'rh_LFJ1': 0.04670206470330096,
                      'rh_LFJ4': -0.25442322842078063, 'rh_LFJ5': 0.08318902699325904, 'rh_RFJ4': 0.012147104952953484,
                      'rh_RFJ1': 0.12789050553406722, 'rh_RFJ2': 0.8443030256522568, 'rh_RFJ3': 0.739851924453851,
                      'rh_MFJ1': 0.24736192156022574, 'rh_MFJ3': 0.6990751351410651, 'rh_MFJ2': 0.9119515783980137,
                      'rh_MFJ4': -0.09907860902502881, 'rh_WRJ2': -0.024970816338612516,
                      'rh_WRJ1': -0.036923804338473525}

hand_joints_goal_20 = {'rh_FFJ1': 0.20499011244971338, 'rh_FFJ2': 1.2075734473544302, 'rh_FFJ3': 1.0,
                      'rh_FFJ4': -0.2599461682016951, 'rh_THJ4': 1.233917619070688, 'rh_THJ5': 0.2141263757057043,
                      'rh_THJ1': 0.8791232663634428, 'rh_THJ2': 0.01802878461513861, 'rh_THJ3': 0.21624349759882638,
                      'rh_LFJ2': 1.2804972588049537, 'rh_LFJ3': 1.0, 'rh_LFJ1': 0.02711732789223925,
                      'rh_LFJ4': -0.2530557722000993, 'rh_LFJ5': 0.09053392795994067, 'rh_RFJ4': 0.024129610613737366,
                      'rh_RFJ1': 0.07598200622906348, 'rh_RFJ2': 0.936587309851457, 'rh_RFJ3': 1.0,
                      'rh_MFJ1': 0.21608074470922445, 'rh_MFJ3': 1.0, 'rh_MFJ2': 0.9641069251861255,
                      'rh_MFJ4': -0.10363242162575106, 'rh_WRJ2': -0.024659666441088357,
                      'rh_WRJ1': -0.03622438391480098}

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
