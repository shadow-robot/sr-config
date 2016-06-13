#!/usr/bin/env python

# This example slowly closes the hand from an open position while monitoring the optoforce  force data
# if the force along the Z axis of any of the sensors goes over a threshold, the hand stops moving and
# returns to an open position

import rospy

from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
from geometry_msgs.msg import WrenchStamped


def read_tactile_values():
    # Assign Z force values from each tactile to the associated finger in the
    # tactile_values dictionary
    tactile_values['FF'] = tactile_0[2]
    tactile_values['MF'] = tactile_1[2]
    tactile_values['RF'] = tactile_2[2]
    tactile_values['LF'] = tactile_3[2]
    tactile_values['TH'] = tactile_4[2]
    return


# Callback function for each tactile, called whenever a new message is received
# on the subscribed topic


def tactile_0_callback(tactile_msg):
    tactile_0[0] = tactile_msg.wrench.force.x
    tactile_0[1] = tactile_msg.wrench.force.y
    tactile_0[2] = tactile_msg.wrench.force.z


def tactile_1_callback(tactile_msg):
    tactile_1[0] = tactile_msg.wrench.force.x
    tactile_1[1] = tactile_msg.wrench.force.y
    tactile_1[2] = tactile_msg.wrench.force.z


def tactile_2_callback(tactile_msg):
    tactile_2[0] = tactile_msg.wrench.force.x
    tactile_2[1] = tactile_msg.wrench.force.y
    tactile_2[2] = tactile_msg.wrench.force.z


def tactile_3_callback(tactile_msg):
    tactile_3[0] = tactile_msg.wrench.force.x
    tactile_3[1] = tactile_msg.wrench.force.y
    tactile_3[2] = tactile_msg.wrench.force.z


def tactile_4_callback(tactile_msg):
    tactile_4[0] = tactile_msg.wrench.force.x
    tactile_4[1] = tactile_msg.wrench.force.y
    tactile_4[2] = tactile_msg.wrench.force.z


rospy.init_node("sr_optoforce_example", anonymous=True)

# Hand finder is used to detect the serial number of the hand and whether the
# hand is left or right
hand_finder = HandFinder()

hand_parameters = hand_finder.get_hand_parameters()
hand_serial = hand_parameters.mapping.keys()[0]

hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                 hand_serial=hand_serial)

tactile_values = {"FF": 0, "MF": 0, "RF": 0, "LF": 0, "TH": 0}
# Create empty dictionaries for x, y, z values for each optoforce sensor
tactile_0 = [None] * 3
tactile_1 = [None] * 3
tactile_2 = [None] * 3
tactile_3 = [None] * 3
tactile_4 = [None] * 3

# A listener for each optoforce topic
optoforce_listener_0 = rospy.Subscriber(
    "optoforce_0", WrenchStamped, tactile_0_callback, queue_size=1)
optoforce_listener_1 = rospy.Subscriber(
    "optoforce_1", WrenchStamped, tactile_1_callback, queue_size=1)
optoforce_listener_2 = rospy.Subscriber(
    "optoforce_2", WrenchStamped, tactile_2_callback, queue_size=1)
optoforce_listener_3 = rospy.Subscriber(
    "optoforce_3", WrenchStamped, tactile_3_callback, queue_size=1)
optoforce_listener_4 = rospy.Subscriber(
    "optoforce_4", WrenchStamped, tactile_4_callback, queue_size=1)

# Define open and closed hand poses
open_pose = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': -8.0,
             'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0,
             'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ1': 0.0, 'rh_LFJ4': -8.0, 'rh_LFJ5': 0.0,
             'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': -3.0,
             'rh_MFJ1': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ4': -3.0,
             'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

pack_pose = {'rh_FFJ1': 90, 'rh_FFJ2': 90, 'rh_FFJ3': 90, 'rh_FFJ4': 4,
             'rh_MFJ1': 90, 'rh_MFJ2': 90, 'rh_MFJ3': 90, 'rh_MFJ4': 1,
             'rh_RFJ1': 90, 'rh_RFJ2': 90, 'rh_RFJ3': 90, 'rh_RFJ4': 1,
             'rh_LFJ1': 90, 'rh_LFJ2': 90, 'rh_LFJ3': 90, 'rh_LFJ4': 4, 'rh_LFJ5': 0.0}

# Move hand to open pose
hand_commander.move_to_joint_value_target_unsafe(open_pose, 3.0, False, angle_degrees=True)
rospy.sleep(3)

# Move hand to a closed pose over a 5s interval
hand_commander.move_to_joint_value_target_unsafe(pack_pose, 5.0, False, angle_degrees=True)

while True:
    # Check if any of the tactile senors have been triggered
    # If so, send the Hand to its start position
    read_tactile_values()

    # Print force values for each finger to screen
    rospy.loginfo("TH force Z: " + str(tactile_values['TH']))
    rospy.loginfo("FF force Z: " + str(tactile_values['FF']))
    rospy.loginfo("MF force Z: " + str(tactile_values['MF']))
    rospy.loginfo("RF force Z: " + str(tactile_values['RF']))
    rospy.loginfo("LF force Z: " + str(tactile_values['LF']))

    # The threshold value to trigger the hand to open is 0.2
    if (tactile_values['FF'] > 0.2 or
        tactile_values['MF'] > 0.2 or
        tactile_values['RF'] > 0.2 or
        tactile_values['LF'] > 0.2 or
        tactile_values['TH'] > 0.2):

        hand_commander.move_to_joint_value_target_unsafe(open_pose, 2.0, False, angle_degrees=True)

        print 'Finger touched.'
        rospy.sleep(2.0)
        break
