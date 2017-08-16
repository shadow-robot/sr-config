#!/usr/bin/env python
#
# Copyright 2014 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import rospy
import random
import time
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder

rospy.init_node("left_hand_demo", anonymous=True)

hand_finder = HandFinder()

hand_parameters = hand_finder.get_hand_parameters()
hand_serial = hand_parameters.mapping.keys()[0]

hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                 hand_serial=hand_serial)

rospy.sleep(0.5)

##########
# RANGES #
##########

inter_time_max = 4.0

# Minimum allowed range for the joints in this particular script
min_range = {"lh_THJ2": -40, "lh_THJ4": 0, "lh_THJ5": -55,
             "lh_FFJ1": 0, "lh_FFJ2": 20, "lh_FFJ3": 0, "lh_FFJ4": -20,
             "lh_MFJ1": 0, "lh_MFJ2": 20, "lh_MFJ3": 0, "lh_MFJ4": -10,
             "lh_RFJ1": 0, "lh_RFJ2": 20, "lh_RFJ3": 0, "lh_RFJ4": -10}

# Maximum allowed range for the joints in this particular script
max_range = {"lh_THJ2": 20, "lh_THJ4": 70, "lh_THJ5": 0,
             "lh_FFJ1": 20, "lh_FFJ2": 90, "lh_FFJ3": 90, "lh_FFJ4": 0,
             "lh_MFJ1": 20, "lh_MFJ2": 90, "lh_MFJ3": 90, "lh_MFJ4": 0,
             "lh_RFJ1": 20, "lh_RFJ2": 90, "lh_RFJ3": 90, "lh_RFJ4": 0}

####################
# POSE DEFINITIONS #
####################

# starting position for the hand 
start_pos = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ4": 0, "lh_THJ5": 0,
             "lh_FFJ1": 0, "lh_FFJ2": 0, "lh_FFJ3": 0, "lh_FFJ4": 0,
             "lh_MFJ1": 0, "lh_MFJ2": 0, "lh_MFJ3": 0, "lh_MFJ4": 0,
             "lh_RFJ1": 0, "lh_RFJ2": 0, "lh_RFJ3": 0, "lh_RFJ4": 0}
# Start position for the Hand
pregrasp_pos = {"lh_THJ2": 12, "lh_THJ4": 69, "lh_THJ5": -23,
                "lh_FFJ1": 0, "lh_FFJ2": 40, "lh_FFJ3": 21, "lh_FFJ4": -15,
                "lh_MFJ1": 0, "lh_MFJ2": 40, "lh_MFJ3": 21, "lh_MFJ4": 0,
                "lh_RFJ1": 0, "lh_RFJ2": 40, "lh_RFJ3": 21, "lh_RFJ4": -7}
# Close position for the Hand
grasp_pos = {"lh_THJ2": 30, "lh_THJ4": 69, "lh_THJ5": -3,
             "lh_FFJ1": 0, "lh_FFJ2": 77, "lh_FFJ3": 67, "lh_FFJ4": -19,
             "lh_MFJ1": 0, "lh_MFJ2": 82, "lh_MFJ3": 62, "lh_MFJ4": 0,
             "lh_RFJ1": 0, "lh_RFJ2": 89, "lh_RFJ3": 64, "lh_RFJ4": -18}
# Random position for the Hand (initialised at 0)
rand_pos = {"lh_THJ2": 0, "lh_THJ4": 0, "lh_THJ5": 0,
            "lh_FFJ1": 0, "lh_FFJ2": 0, "lh_FFJ3": 0, "lh_FFJ4": 0,
            "lh_MFJ1": 0, "lh_MFJ2": 0, "lh_MFJ3": 0, "lh_MFJ4": 0,
            "lh_RFJ1": 0, "lh_RFJ2": 0, "lh_RFJ3": 0, "lh_RFJ4": 0}
# flex first finger
flex_ff = {"lh_FFJ1": 90, "lh_FFJ2": 90, "lh_FFJ3": 90, "lh_FFJ4": 0}
# extend first finger
ext_ff = {"lh_FFJ1": 0, "lh_FFJ2": 0, "lh_FFJ3": 0, "lh_FFJ4": 0}
# flex middle finger
flex_mf = {"lh_MFJ1": 90, "lh_MFJ2": 90, "lh_MFJ3": 90, "lh_MFJ4": 0}
# extend middle finger
ext_mf = {"lh_MFJ1": 0, "lh_MFJ2": 0, "lh_MFJ3": 0, "lh_MFJ4": 0}
# flex ring finger
flex_rf = {"lh_RFJ1": 90, "lh_RFJ2": 90, "lh_RFJ3": 90, "lh_RFJ4": 0}
# extend ring finger
ext_rf = {"lh_RFJ1": 0, "lh_RFJ2": 0, "lh_RFJ3": 0, "lh_RFJ4": 0}
# flex thumb step 1
flex_th_1 = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ4": 70, "lh_THJ5": 0}
# flex thumb step 2
flex_th_2 = {"lh_THJ1": 35, "lh_THJ2": 38, "lh_THJ4": 70, "lh_THJ5": 58}
# extend thumb step 1
ext_th_1 = {"lh_THJ1": 10, "lh_THJ2": 20, "lh_THJ4": 35, "lh_THJ5": 25}
# extend thumb step 2
ext_th_2 = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ4": 0, "lh_THJ5": 0}
# zero thumb
zero_th = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ4": 0, "lh_THJ5": 0}
# Pre O.K. with first finger
pre_ff_ok = {"lh_THJ4": 70}
# O.K. with first finger
ff_ok = {"lh_THJ1": 18, "lh_THJ2": 20, "lh_THJ4": 56, "lh_THJ5": 18,
         "lh_FFJ1": 0, "lh_FFJ2": 75, "lh_FFJ3": 48, "lh_FFJ4": -0.2,
         "lh_MFJ1": 0, "lh_MFJ2": 42, "lh_MFJ3": 33, "lh_MFJ4": -3,
         "lh_RFJ1": 0, "lh_RFJ2": 50, "lh_RFJ3": 18, "lh_RFJ4": 0.5}
# O.K. transition from first finger to middle finger
ff2mf_ok = {"lh_THJ1": 5, "lh_THJ2": 12, "lh_THJ4": 60, "lh_THJ5": 2,
            "lh_FFJ1": 0, "lh_FFJ2": 14, "lh_FFJ3": 7, "lh_FFJ4": -0.4,
            "lh_MFJ1": 0, "lh_MFJ2": 42, "lh_MFJ3": 33, "lh_MFJ4": -3,
            "lh_RFJ1": 0, "lh_RFJ2": 50, "lh_RFJ3": 18, "lh_RFJ4": 0.5}
# O.K. with middle finger
mf_ok = {"lh_THJ1": 27, "lh_THJ2": 16, "lh_THJ4": 68, "lh_THJ5": 27,
         "lh_FFJ1": 0, "lh_FFJ2": 14, "lh_FFJ3": 7, "lh_FFJ4": -0.4,
         "lh_MFJ1": 8, "lh_MFJ2": 86, "lh_MFJ3": 43, "lh_MFJ4": 9,
         "lh_RFJ1": 0, "lh_RFJ2": 50, "lh_RFJ3": 18, "lh_RFJ4": -10}
# O.K. transition from middle finger to ring finger
mf2rf_ok = {"lh_THJ1": 5, "lh_THJ2": -5, "lh_THJ4": 70, "lh_THJ5": 19,
            "lh_FFJ1": 0, "lh_FFJ2": 14, "lh_FFJ3": 7, "lh_FFJ4": -0.4,
            "lh_MFJ1": 0, "lh_MFJ2": 45, "lh_MFJ3": 4, "lh_MFJ4": -1,
            "lh_RFJ1": 0, "lh_RFJ2": 50, "lh_RFJ3": 18, "lh_RFJ4": -19}
# O.K. with ring finger
rf_ok = {"lh_THJ1": 32, "lh_THJ2": 33, "lh_THJ4": 70, "lh_THJ5": 27,
         "lh_FFJ1": 0, "lh_FFJ2": 14, "lh_FFJ3": 7, "lh_FFJ4": -0.4,
         "lh_MFJ1": 0, "lh_MFJ2": 45, "lh_MFJ3": 4, "lh_MFJ4": -1,
         "lh_RFJ1": 29, "lh_RFJ2": 90, "lh_RFJ3": 36, "lh_RFJ4": -20}
# O.K. transition from ring finger to little finger
rf2lf_ok = {"lh_THJ1": 5, "lh_THJ2": 4.5, "lh_THJ4": 60, "lh_THJ5": 21,
            "lh_FFJ1": 0, "lh_FFJ2": 14, "lh_FFJ3": 7, "lh_FFJ4": -0.4,
            "lh_MFJ1": 0, "lh_MFJ2": 45, "lh_MFJ3": 4, "lh_MFJ4": -1,
            "lh_RFJ1": 0, "lh_RFJ2": 30, "lh_RFJ3": 6, "lh_RFJ4": 0.5}
# O.K. with little finger
lf_ok = {"lh_THJ1": 24, "lh_THJ2": 11, "lh_THJ4": 69, "lh_THJ5": 22,
         "lh_FFJ1": 0, "lh_FFJ2": 14, "lh_FFJ3": 7, "lh_FFJ4": -0.4,
         "lh_MFJ1": 0, "lh_MFJ2": 15, "lh_MFJ3": 4, "lh_MFJ4": -1,
         "lh_RFJ1": 0, "lh_RFJ2": 15, "lh_RFJ3": 6, "lh_RFJ4": 0.5}
# lateral rf ext side
l_ext_rf = {"lh_RFJ4": -15}
# lateral mf ext side
l_ext_mf = {"lh_MFJ4": 15}
# lateral ff ext side
l_ext_ff = {"lh_FFJ4": 15}
# lateral all int side
l_int_all = {"lh_FFJ4": -15, "lh_MFJ4": -15, "lh_RFJ4": 15}
# lateral all ext side
l_ext_all = {"lh_FFJ4": 15, "lh_MFJ4": 15, "lh_RFJ4": -15}
# lateral ff int side
l_int_ff = {"lh_FFJ4": -15}
# lateral mf int side
l_int_mf = {"lh_MFJ4": -15}
# lateral rf int side
l_int_rf = {"lh_RFJ4": 15}
# all zero
l_zero_all = {"lh_FFJ4": 0, "lh_MFJ4": 0, "lh_RFJ4": 0}
# spock
l_spock = {"lh_FFJ4": -20, "lh_MFJ4": -20, "lh_RFJ4": -20}
# grasp for shaking hands step 1
shake_grasp_1 = {"lh_THJ1": 0, "lh_THJ2": 6, "lh_THJ4": 37, "lh_THJ5": 9,
                 "lh_FFJ1": 0, "lh_FFJ2": 21, "lh_FFJ3": 26, "lh_FFJ4": 0,
                 "lh_MFJ1": 0, "lh_MFJ2": 18, "lh_MFJ3": 24, "lh_MFJ4": 0,
                 "lh_RFJ1": 0, "lh_RFJ2": 30, "lh_RFJ3": 16, "lh_RFJ4": 0}
# grasp for shaking hands step 2
shake_grasp_2 = {"lh_THJ1": 21, "lh_THJ2": 21, "lh_THJ4": 42, "lh_THJ5": 21,
                 "lh_FFJ1": 0, "lh_FFJ2": 75, "lh_FFJ3": 29, "lh_FFJ4": 0,
                 "lh_MFJ1": 0, "lh_MFJ2": 75, "lh_MFJ3": 41, "lh_MFJ4": 0,
                 "lh_RFJ1": 0, "lh_RFJ2": 75, "lh_RFJ3": 41, "lh_RFJ4": 0}
# store step 1 PST
store_1_PST = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ4": 60, "lh_THJ5": 0,
               "lh_FFJ1": 90, "lh_FFJ2": 90, "lh_FFJ3": 90, "lh_FFJ4": 0,
               "lh_MFJ1": 90, "lh_MFJ2": 90, "lh_MFJ3": 90, "lh_MFJ4": 0,
               "lh_RFJ1": 90, "lh_RFJ2": 90, "lh_RFJ3": 90, "lh_RFJ4": 0}
# store step 2 PST
store_2_PST = {"lh_THJ1": 50, "lh_THJ2": 12, "lh_THJ4": 60, "lh_THJ5": 27,
               "lh_FFJ1": 90, "lh_FFJ2": 90, "lh_FFJ3": 90, "lh_FFJ4": 0,
               "lh_MFJ1": 90, "lh_MFJ2": 90, "lh_MFJ3": 90, "lh_MFJ4": 0,
               "lh_RFJ1": 90, "lh_RFJ2": 90, "lh_RFJ3": 90, "lh_RFJ4": 0}
# store step 1 Bio_Tac
store_1_BioTac = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ4": 30, "lh_THJ5": 0,
                  "lh_FFJ1": 90, "lh_FFJ2": 90, "lh_FFJ3": 90, "lh_FFJ4": 0,
                  "lh_MFJ1": 90, "lh_MFJ2": 90, "lh_MFJ3": 90, "lh_MFJ4": 0,
                  "lh_RFJ1": 90, "lh_RFJ2": 90, "lh_RFJ3": 90, "lh_RFJ4": 0}
# store step 2 Bio_Tac
store_2_BioTac = {"lh_THJ1": 20, "lh_THJ2": 36, "lh_THJ4": 30, "lh_THJ5": -3,
                  "lh_FFJ1": 90, "lh_FFJ2": 90, "lh_FFJ3": 90, "lh_FFJ4": 0,
                  "lh_MFJ1": 90, "lh_MFJ2": 90, "lh_MFJ3": 90, "lh_MFJ4": 0,
                  "lh_RFJ1": 90, "lh_RFJ2": 90, "lh_RFJ3": 90, "lh_RFJ4": 0}
# store step 3
store_3 = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ4": 65, "lh_THJ5": 0}
# business card pre-zero position 
bc_pre_zero = {"lh_THJ1": 15, "lh_THJ2": 7, "lh_THJ4": 48, "lh_THJ5": -17,
               "lh_FFJ1": 0, "lh_FFJ2": 14, "lh_FFJ3": 7, "lh_FFJ4": -1,
               "lh_MFJ1": 0, "lh_MFJ2": 51, "lh_MFJ3": 33, "lh_MFJ4": 20,
               "lh_RFJ1": 0, "lh_RFJ2": 50, "lh_RFJ3": 18, "lh_RFJ4": -20}
# business card zero position 
bc_zero = {"lh_THJ1": 23, "lh_THJ2": 9, "lh_THJ4": 44, "lh_THJ5": -14,
           "lh_MFJ1": 0, "lh_MFJ2": 63, "lh_MFJ3": 23, "lh_MFJ4": 20}
# business card position 1 
bc_1 = {"lh_FFJ1": 47, "lh_FFJ2": 90, "lh_FFJ3": 7}
# business card position 2 
bc_2 = {"lh_FFJ1": 47, "lh_FFJ2": 90, "lh_FFJ3": 58}
# business card position 3 
bc_3 = {"lh_FFJ1": 0, "lh_FFJ2": 70, "lh_FFJ3": 58}
# business card position 4 
bc_4 = {"lh_FFJ1": 90, "lh_FFJ2": 90, "lh_FFJ3": 58}
# business card position 5 
bc_5 = {"lh_FFJ1": 90, "lh_FFJ2": 90, "lh_FFJ3": 0}
# business card position 6 
bc_6 = {"lh_FFJ1": 0, "lh_FFJ2": 0, "lh_FFJ3": 0}
# business card position 7 
bc_7 = {"lh_FFJ1": 47, "lh_FFJ2": 90, "lh_FFJ3": 15}
# business card position 8 
bc_8 = {"lh_FFJ1": 47, "lh_FFJ2": 90, "lh_FFJ3": 58}
# business card position 9 
bc_9 = {"lh_FFJ1": 0, "lh_FFJ2": 71, "lh_FFJ3": 58}
# business card position 10 
bc_10 = {"lh_MFJ3": 64, "lh_FFJ4": 20}
# business card position 11 
bc_11 = {"lh_FFJ1": 0, "lh_FFJ2": 70, "lh_FFJ3": 57, "lh_FFJ4": 20,
         "lh_THJ4": 57, "lh_THJ5": 20,}
# business card position 12 
bc_12 = {"lh_MFJ1": 0, "lh_MFJ2": 20, "lh_MFJ3": 10, "lh_MFJ4": 0}


########################
# FUNCTION DEFINITIONS #
########################

def sequence_ff():
    # Start sequence 1
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(store_3, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 1.2, False, angle_degrees=True)
    rospy.sleep(1.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, 1.0, False, angle_degrees=True)
    rospy.sleep(1.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_th_1, 0.7, False, angle_degrees=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(flex_th_2, 1.0, False, angle_degrees=True)
    rospy.sleep(1.0)
    hand_commander.move_to_joint_value_target_unsafe(ext_th_2, 1.7, False, angle_degrees=True)
    rospy.sleep(1.9)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_rf, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_mf, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_ff, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_all, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_all, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_ff, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_mf, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_rf, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_zero_all, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_spock, 1.0, False, angle_degrees=True)
    rospy.sleep(1.0)
    hand_commander.move_to_joint_value_target_unsafe(l_zero_all, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(pre_ff_ok, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ff_ok, 1.0, False, angle_degrees=True)
    rospy.sleep(1.7)
    hand_commander.move_to_joint_value_target_unsafe(ff2mf_ok, 0.8, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(mf_ok, 1.0, False, angle_degrees=True)
    rospy.sleep(1.7)
    hand_commander.move_to_joint_value_target_unsafe(mf2rf_ok, 0.8, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(rf_ok, 1.0, False, angle_degrees=True)
    rospy.sleep(1.7)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 1.0, False, angle_degrees=True)
    rospy.sleep(1.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(pre_ff_ok, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ff_ok, 2.0, False, angle_degrees=True)
    rospy.sleep(3.0)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 1.5, False, angle_degrees=True)
    rospy.sleep(1.5)
    return


def sequence_mf():
    # Start the sequence 2
    rospy.sleep(0.5)
    # Initialize wake time
    wake_time = time.time()

    while True:
        # Check if any of the tactile senors have been triggered
        # If so, send the Hand to its start position
        read_tactile_values()
        if (tactile_values['FF'] > force_zero['FF'] or
                    tactile_values['MF'] > force_zero['MF'] or
                    tactile_values['RF'] > force_zero['RF'] or
                    tactile_values['LF'] > force_zero['LF'] or
                    tactile_values['TH'] > force_zero['TH']):

            hand_commander.move_to_joint_value_target_unsafe(start_pos, 0.7, False, angle_degrees=True)
            print 'HAND TOUCHED!'
            rospy.sleep(1.2)

            if tactile_values['TH'] > force_zero['TH']:
                break

        # If the tactile sensors have not been triggered and the Hand
        # is not in the middle of a movement, generate a random position
        # and interpolation time
        else:
            if time.time() > wake_time:
                for i in rand_pos:
                    rand_pos[i] = random.randrange(min_range[i], max_range[i])

                rand_pos['lh_FFJ4'] = random.randrange(min_range['lh_FFJ4'], rand_pos['lh_MFJ4'])
                #            rand_pos['lh_LFJ4'] = random.randrange(min_range['lh_LFJ4'],rand_pos['lh_RFJ4'])
                inter_time = inter_time_max * random.random()
                #            rand_pos['interpolation_time'] = max_range['interpolation_time'] * random.random()

                hand_commander.move_to_joint_value_target_unsafe(rand_pos, inter_time, False, angle_degrees=True)
                wake_time = time.time() + inter_time * 0.9
    return


def sequence_rf():
    # Start the sequence 3
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(bc_pre_zero, 2.0, False, angle_degrees=True)
    rospy.sleep(2)
    hand_commander.move_to_joint_value_target_unsafe(bc_zero, 1.0, False, angle_degrees=True)
    rospy.sleep(5)
    hand_commander.move_to_joint_value_target_unsafe(bc_1, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(bc_2, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(bc_3, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(bc_4, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(bc_5, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(bc_6, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(bc_7, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(bc_8, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(bc_9, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(bc_11, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(bc_12, 0.5, False, angle_degrees=True)
    rospy.sleep(3)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 1.5, False, angle_degrees=True)
    rospy.sleep(1.5)

    return


def sequence_lf():
    # Start the sequence 4
    # Trigger flag array
    trigger = [0, 0, 0, 0, 0]

    # Move Hand to zero position
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 2.0, False, angle_degrees=True)
    rospy.sleep(2.0)

    # Move Hand to starting position
    hand_commander.move_to_joint_value_target_unsafe(pregrasp_pos, 2.0, False, angle_degrees=True)
    rospy.sleep(2.0)

    # Move Hand to close position
    hand_commander.move_to_joint_value_target_unsafe(grasp_pos, 11.0, False, angle_degrees=True)
    offset1 = 3

    # Initialize end time
    end_time = time.time() + 11

    while True:
        # Check the state of the tactile senors
        read_tactile_values()

        # Record current joint positions
        hand_pos = hand_commander.get_joints_position()

        # If any tactile sensor has been triggered, send
        # the corresponding digit to its current position
        if tactile_values['FF'] > force_zero['FF'] and trigger[0] == 0:
            hand_pos_incr_f = {"lh_FFJ1": hand_pos['lh_FFJ1'] + offset1, "lh_FFJ3": hand_pos['lh_FFJ3'] + offset1}
            hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_f, 0.5, False, angle_degrees=True)
            print 'First finger contact'
            trigger[0] = 1

        if tactile_values['MF'] > force_zero['MF'] and trigger[1] == 0:
            hand_pos_incr_m = {"lh_MFJ1": hand_pos['lh_MFJ1'] + offset1, "lh_MFJ3": hand_pos['lh_MFJ3'] + offset1}
            hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_m, 0.5, False, angle_degrees=True)
            print 'Middle finger contact'
            trigger[1] = 1

        if tactile_values['RF'] > force_zero['RF'] and trigger[2] == 0:
            hand_pos_incr_r = {"lh_RFJ1": hand_pos['lh_RFJ1'] + offset1, "lh_RFJ3": hand_pos['lh_RFJ3'] + offset1}
            hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_r, 0.5, False, angle_degrees=True)
            print 'Ring finger contact'
            trigger[2] = 1

        #      if ( tactile_values['LF'] > force_zero['LF'] and trigger[3] == 0 ):
        #         hand_pos_incr_l = {"lh_LFJ1": hand_pos['lh_LFJ1'] + offset1, "lh_LFJ3": hand_pos['lh_LFJ3'] + offset1}
        #         hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_l, 0.5, False, angle_degrees=True)
        #         print 'Little finger contact'
        #         trigger[3] = 1

        if tactile_values['TH'] > force_zero['TH'] and trigger[4] == 0:
            hand_pos_incr_th = {"lh_THJ2": hand_pos['lh_THJ2'] + offset1, "lh_THJ5": hand_pos['lh_THJ5'] + offset1}
            hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_th, 0.5, False, angle_degrees=True)
            print 'Thumb contact'
            trigger[4] = 1

        if (trigger[0] == 1 and
                    trigger[1] == 1 and
                    trigger[2] == 1 and
                    trigger[3] == 1 and
                    trigger[4] == 1):
            break

        if time.time() > end_time:
            break

    # Send all joints to current position to compensate
    # for minor offsets created in the previous loop
    hand_pos = hand_commander.get_joints_position()
    hand_commander.move_to_joint_value_target_unsafe(hand_pos, 2.0, False, angle_degrees=True)
    rospy.sleep(2.0)

    # Generate new values to squeeze object slightly
    offset2 = 3
    squeeze = hand_pos.copy()
    squeeze.update({"lh_THJ5": hand_pos['lh_THJ5'] + offset2, "lh_THJ2": hand_pos['lh_THJ2'] + offset2,
                    "lh_FFJ3": hand_pos['lh_FFJ3'] + offset2, "lh_FFJ1": hand_pos['lh_FFJ1'] + offset2,
                    "lh_MFJ3": hand_pos['lh_MFJ3'] + offset2, "lh_MFJ1": hand_pos['lh_MFJ1'] + offset2,
                    "lh_RFJ3": hand_pos['lh_RFJ3'] + offset2, "lh_RFJ1": hand_pos['lh_RFJ1'] + offset2})

    # Squeeze object gently
    hand_commander.move_to_joint_value_target_unsafe(squeeze, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(hand_pos, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(squeeze, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(hand_pos, 2.0, False, angle_degrees=True)
    rospy.sleep(2.0)
    hand_commander.move_to_joint_value_target_unsafe(pregrasp_pos, 2.0, False, angle_degrees=True)
    rospy.sleep(2.0)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 2.0, False, angle_degrees=True)
    rospy.sleep(2.0)
    return


def sequence_th():
    # Start the sequence 5
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 1.5, False, angle_degrees=True)
    rospy.sleep(1.5)
    return


def zero_tactile_sensors():
    # Move Hand to zero position
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 1.0, False, angle_degrees=True)

    print '\n\nPLEASE ENSURE THAT THE TACTILE SENSORS ARE NOT PRESSED\n'
    # raw_input ('Press ENTER to continue')
    rospy.sleep(1.0)

    for x in xrange(1, 1000):
        # Read current state of tactile sensors to zero them
        read_tactile_values()

        if tactile_values['FF'] > force_zero['FF']:
            force_zero['FF'] = tactile_values['FF']
        if tactile_values['MF'] > force_zero['MF']:
            force_zero['MF'] = tactile_values['MF']
        if tactile_values['RF'] > force_zero['RF']:
            force_zero['RF'] = tactile_values['RF']
        if tactile_values['LF'] > force_zero['LF']:
            force_zero['LF'] = tactile_values['LF']
        if tactile_values['TH'] > force_zero['TH']:
            force_zero['TH'] = tactile_values['TH']

    force_zero['FF'] += 50
    force_zero['MF'] += 50
    force_zero['RF'] += 50
    force_zero['LF'] += 50
    force_zero['TH'] += 50

    print 'Force Zero', force_zero

    rospy.loginfo("\n\nOK, ready for the demo")

    print "\nPRESS ONE OF THE TACTILES TO START A DEMO"
    print "   FF: Standard Demo"
    print "   MF: Shy Hand Demo"
    print "   RF: Card Trick Demo"
    #   print "   LF: Grasp Demo"
    print "   TH: Open Hand"

    return


def read_tactile_values():
    # Read tactile type
    tactile_type = hand_commander.get_tactile_type()
    # Read current state of tactile sensors
    tactile_state = hand_commander.get_tactile_state()

    if tactile_type == "biotac":
        tactile_values['FF'] = tactile_state.tactiles[0].pdc
        tactile_values['MF'] = tactile_state.tactiles[1].pdc
        tactile_values['RF'] = tactile_state.tactiles[2].pdc
        tactile_values['LF'] = tactile_state.tactiles[3].pdc
        tactile_values['TH'] = tactile_state.tactiles[4].pdc

    elif tactile_type == "PST":
        tactile_values['FF'] = tactile_state.pressure[0]
        tactile_values['MF'] = tactile_state.pressure[1]
        tactile_values['RF'] = tactile_state.pressure[2]
        tactile_values['LF'] = tactile_state.pressure[3]
        tactile_values['TH'] = tactile_state.pressure[4]

    elif tactile_type is None:
        print "You don't have tactile sensors. Talk to your Shadow representative to purchase some"

    return


########
# MAIN #
########

# Zero values in dictionary for tactile sensors (initialized at 0)
force_zero = {"FF": 0, "MF": 0, "RF": 0, "LF": 0, "TH": 0}
# Initialize values for current tactile values
tactile_values = {"FF": 0, "MF": 0, "RF": 0, "LF": 0, "TH": 0}
# Zero tactile sensors
zero_tactile_sensors()

while not rospy.is_shutdown():
    # Check the state of the tactile senors
    read_tactile_values()

    # If the tactile is touched, trigger the corresponding function
    if tactile_values['FF'] > force_zero['FF']:
        print 'First finger contact'
        sequence_ff()
        print 'FF demo completed'
        zero_tactile_sensors()
    if tactile_values['MF'] > force_zero['MF']:
        print 'Middle finger contact'
        sequence_mf()
        print 'MF demo completed'
        zero_tactile_sensors()
    if tactile_values['RF'] > force_zero['RF']:
        print 'Ring finger contact'
        sequence_rf()
        print 'RF demo completed'
        zero_tactile_sensors()
    #   if (tactile_values['LF'] > force_zero['LF']):
    #      print 'Little finger contact'
    #      sequence_lf()
    #      print 'LF demo completed'
    #      zero_tactile_sensors()
    if tactile_values['TH'] > force_zero['TH']:
        print 'Thumb finger contact'
        sequence_th()
        print 'TH demo completed'
        zero_tactile_sensors()



