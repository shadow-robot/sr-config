#!/usr/bin/env python3


# Copyright 2019 Shadow Robot Company Ltd.
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

from __future__ import absolute_import
import rospy
import random
import time
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("left_hand_demo", anonymous=True)

hand_commander = SrHandCommander(name="left_hand")

##########
# RANGES #
##########

inter_time_max = 4.0

# Minimum alllowed range for the joints in this particular script
min_range = {"lh_THJ2": -40, "lh_THJ3": -12, "lh_THJ4": 0, "lh_THJ5": -55,
             "lh_FFJ1": 0, "lh_FFJ2": 20, "lh_FFJ3": 0, "lh_FFJ4": -20,
             "lh_MFJ1": 0, "lh_MFJ2": 20, "lh_MFJ3": 0, "lh_MFJ4": -10,
             "lh_RFJ1": 0, "lh_RFJ2": 20, "lh_RFJ3": 0, "lh_RFJ4": -10,
             "lh_LFJ1": 10, "lh_LFJ2": 10, "lh_LFJ3": 0, "lh_LFJ4": -20, "lh_LFJ5": 0,
             "lh_WRJ1": -20, "lh_WRJ2": -10}

# Maximum alllowed range for the joints in this particular script
max_range = {"lh_THJ2": 20, "lh_THJ3": 12, "lh_THJ4": 70, "lh_THJ5": 0,
             "lh_FFJ1": 20, "lh_FFJ2": 90, "lh_FFJ3": 90, "lh_FFJ4": 0,
             "lh_MFJ1": 20, "lh_MFJ2": 90, "lh_MFJ3": 90, "lh_MFJ4": 0,
             "lh_RFJ1": 20, "lh_RFJ2": 90, "lh_RFJ3": 90, "lh_RFJ4": 0,
             "lh_LFJ1": 20, "lh_LFJ2": 90, "lh_LFJ3": 90, "lh_LFJ4": 0, "lh_LFJ5": 1,
             "lh_WRJ1": 10, "lh_WRJ2": 5}

####################
# POSE DEFINITIONS #
####################

# starting position for the hand
start_pos = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ3": 0, "lh_THJ4": 0, "lh_THJ5": 0,
             "lh_FFJ1": 0, "lh_FFJ2": 0, "lh_FFJ3": 0, "lh_FFJ4": 0,
             "lh_MFJ1": 0, "lh_MFJ2": 0, "lh_MFJ3": 0, "lh_MFJ4": 0,
             "lh_RFJ1": 0, "lh_RFJ2": 0, "lh_RFJ3": 0, "lh_RFJ4": 0,
             "lh_LFJ1": 0, "lh_LFJ2": 0, "lh_LFJ3": 0, "lh_LFJ4": 0, "lh_LFJ5": 0,
             "lh_WRJ1": 0, "lh_WRJ2": 0}
# Start position for the Hand
pregrasp_pos = {"lh_THJ2": 12, "lh_THJ3": 15, "lh_THJ4": 69, "lh_THJ5": -23,
                "lh_FFJ1": 0, "lh_FFJ2": 40, "lh_FFJ3": 21, "lh_FFJ4": -15,
                "lh_MFJ1": 0, "lh_MFJ2": 40, "lh_MFJ3": 21, "lh_MFJ4": 0,
                "lh_RFJ1": 0, "lh_RFJ2": 40, "lh_RFJ3": 21, "lh_RFJ4": -7,
                "lh_LFJ1": 0, "lh_LFJ2": 40, "lh_LFJ3": 21, "lh_LFJ4": -10, "lh_LFJ5": 0,
                "lh_WRJ1": 0, "lh_WRJ2": 0}
# Close position for the Hand
grasp_pos = {"lh_THJ2": 30, "lh_THJ3": 15, "lh_THJ4": 69, "lh_THJ5": -3,
             "lh_FFJ1": 0, "lh_FFJ2": 77, "lh_FFJ3": 67, "lh_FFJ4": -19,
             "lh_MFJ1": 0, "lh_MFJ2": 82, "lh_MFJ3": 62, "lh_MFJ4": 0,
             "lh_RFJ1": 0, "lh_RFJ2": 89, "lh_RFJ3": 64, "lh_RFJ4": -18,
             "lh_LFJ1": 7, "lh_LFJ2": 90, "lh_LFJ3": 64, "lh_LFJ4": -19, "lh_LFJ5": 0,
             "lh_WRJ1": 0, "lh_WRJ2": 0}
# Random position for the Hand (initialied at 0)
rand_pos = {"lh_THJ2": 0, "lh_THJ3": 0, "lh_THJ4": 0, "lh_THJ5": 0,
            "lh_FFJ1": 0, "lh_FFJ2": 0, "lh_FFJ3": 0, "lh_FFJ4": 0,
            "lh_MFJ1": 0, "lh_MFJ2": 0, "lh_MFJ3": 0, "lh_MFJ4": 0,
            "lh_RFJ1": 0, "lh_RFJ2": 0, "lh_RFJ3": 0, "lh_RFJ4": 0,
            "lh_LFJ1": 0, "lh_LFJ2": 0, "lh_LFJ3": 0, "lh_LFJ4": 0, "lh_LFJ5": 0,
            "lh_WRJ1": 0, "lh_WRJ2": 0}
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
# flex little finger
flex_lf = {"lh_LFJ1": 90, "lh_LFJ2": 90, "lh_LFJ3": 90, "lh_LFJ4": 0}
# extend middle finger
ext_lf = {"lh_LFJ1": 0, "lh_LFJ2": 0, "lh_LFJ3": 0, "lh_LFJ4": 0}
# flex thumb step 1
flex_th_1 = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ3": 0, "lh_THJ4": 70, "lh_THJ5": 0}
# flex thumb step 2
flex_th_2 = {"lh_THJ1": 35, "lh_THJ2": 38, "lh_THJ3": 10, "lh_THJ4": 70, "lh_THJ5": 58}
# extend thumb step 1
ext_th_1 = {"lh_THJ1": 90, "lh_THJ2": -40, "lh_THJ3": -10, "lh_THJ4": 35, "lh_THJ5": -60}
# extend thumb step 2
ext_th_2 = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ3": 0, "lh_THJ4": 0, "lh_THJ5": 0}
# zero thumb
zero_th = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ3": 0, "lh_THJ4": 0, "lh_THJ5": 0}
# Pre O.K. with first finger
pre_ff_ok = {"lh_THJ4": 70}
# O.K. with first finger
ff_ok = {"lh_THJ1": 17, "lh_THJ2": 20, "lh_THJ3": 0, "lh_THJ4": 56, "lh_THJ5": 18,
         "lh_FFJ1": 0, "lh_FFJ2": 75, "lh_FFJ3": 52, "lh_FFJ4": -0.2,
         "lh_MFJ1": 0, "lh_MFJ2": 42, "lh_MFJ3": 33, "lh_MFJ4": -3,
         "lh_RFJ1": 0, "lh_RFJ2": 50, "lh_RFJ3": 18, "lh_RFJ4": 0.5,
         "lh_LFJ1": 0, "lh_LFJ2": 30, "lh_LFJ3": 0, "lh_LFJ4": -6, "lh_LFJ5": 7}
# O.K. transition from first finger to middle finger
ff2mf_ok = {"lh_THJ1": 5, "lh_THJ2": 12, "lh_THJ3": 4, "lh_THJ4": 60, "lh_THJ5": 2,
            "lh_FFJ1": 0, "lh_FFJ2": 14, "lh_FFJ3": 7, "lh_FFJ4": -0.4,
            "lh_MFJ1": 0, "lh_MFJ2": 42, "lh_MFJ3": 33, "lh_MFJ4": -3,
            "lh_RFJ1": 0, "lh_RFJ2": 50, "lh_RFJ3": 18, "lh_RFJ4": 0.5,
            "lh_LFJ1": 0, "lh_LFJ2": 30, "lh_LFJ3": 0, "lh_LFJ4": -6, "lh_LFJ5": 7}
# O.K. with middle finger
mf_ok = {"lh_THJ1": 19, "lh_THJ2": 17, "lh_THJ3": 6, "lh_THJ4": 66, "lh_THJ5": 31,
         "lh_FFJ1": 0, "lh_FFJ2": 15, "lh_FFJ3": 7, "lh_FFJ4": -0.4,
         "lh_MFJ1": 11, "lh_MFJ2": 71, "lh_MFJ3": 49, "lh_MFJ4": 10,
         "lh_RFJ1": 0, "lh_RFJ2": 50, "lh_RFJ3": 18, "lh_RFJ4": -10,
         "lh_LFJ1": 0, "lh_LFJ2": 30, "lh_LFJ3": 0, "lh_LFJ4": -6, "lh_LFJ5": 7}
# O.K. transition from middle finger to ring finger
mf2rf_ok = {"lh_THJ1": 5, "lh_THJ2": -5, "lh_THJ3": 15, "lh_THJ4": 70, "lh_THJ5": 19,
            "lh_FFJ1": 0, "lh_FFJ2": 14, "lh_FFJ3": 7, "lh_FFJ4": -0.4,
            "lh_MFJ1": 0, "lh_MFJ2": 45, "lh_MFJ3": 4, "lh_MFJ4": -1,
            "lh_RFJ1": 0, "lh_RFJ2": 50, "lh_RFJ3": 18, "lh_RFJ4": -19,
            "lh_LFJ1": 0, "lh_LFJ2": 30, "lh_LFJ3": 0, "lh_LFJ4": -12, "lh_LFJ5": 7}
# O.K. with ring finger
rf_ok = {"lh_THJ1": 8, "lh_THJ2": 15, "lh_THJ3": 15, "lh_THJ4": 70, "lh_THJ5": 45,
         "lh_FFJ1": 0, "lh_FFJ2": 14, "lh_FFJ3": 7, "lh_FFJ4": -0.4,
         "lh_MFJ1": 0, "lh_MFJ2": 45, "lh_MFJ3": 4, "lh_MFJ4": -1,
         "lh_RFJ1": 3, "lh_RFJ2": 90, "lh_RFJ3": 42, "lh_RFJ4": -19,
         "lh_LFJ1": 0, "lh_LFJ2": 30, "lh_LFJ3": 0, "lh_LFJ4": -12, "lh_LFJ5": 7}
# O.K. transition from ring finger to little finger
rf2lf_ok = {"lh_THJ1": 5, "lh_THJ2": 4.5, "lh_THJ3": 8, "lh_THJ4": 60, "lh_THJ5": 21,
            "lh_FFJ1": 0, "lh_FFJ2": 14, "lh_FFJ3": 7, "lh_FFJ4": -0.4,
            "lh_MFJ1": 0, "lh_MFJ2": 45, "lh_MFJ3": 4, "lh_MFJ4": -1,
            "lh_RFJ1": 0, "lh_RFJ2": 30, "lh_RFJ3": 6, "lh_RFJ4": 0.5,
            "lh_LFJ1": 0, "lh_LFJ2": 30, "lh_LFJ3": 0, "lh_LFJ4": -10, "lh_LFJ5": 7}
# O.K. with little finger
lf_ok = {"lh_THJ1": 25, "lh_THJ2": 14, "lh_THJ3": 10, "lh_THJ4": 69, "lh_THJ5": 22,
         "lh_FFJ1": 0, "lh_FFJ2": 14, "lh_FFJ3": 7, "lh_FFJ4": -0.4,
         "lh_MFJ1": 0, "lh_MFJ2": 15, "lh_MFJ3": 4, "lh_MFJ4": -1,
         "lh_RFJ1": 0, "lh_RFJ2": 15, "lh_RFJ3": 6, "lh_RFJ4": 0.5,
         "lh_LFJ1": 0, "lh_LFJ2": 78, "lh_LFJ3": 26, "lh_LFJ4": 5, "lh_LFJ5": 37}
# zero wrist
zero_wr = {"lh_WRJ1": 0, "lh_WRJ2": 0}
# north wrist
n_wr = {"lh_WRJ1": 15, "lh_WRJ2": 0}
# south wrist
s_wr = {"lh_WRJ1": -30, "lh_WRJ2": 0}
# east wrist
e_wr = {"lh_WRJ1": 0, "lh_WRJ2": 8}
# west wrist
w_wr = {"lh_WRJ1": 0, "lh_WRJ2": -14}
# northeast wrist
ne_wr = {"lh_WRJ1": 15, "lh_WRJ2": 8}
# northwest wrist
nw_wr = {"lh_WRJ1": 15, "lh_WRJ2": -14}
# southweast wrist
sw_wr = {"lh_WRJ1": -30, "lh_WRJ2": -14}
# southeast wrist
se_wr = {"lh_WRJ1": -30, "lh_WRJ2": 8}
# lateral lf ext side
l_ext_lf = {"lh_LFJ4": -15}
# lateral rf ext side
l_ext_rf = {"lh_RFJ4": -15}
# lateral mf ext side
l_ext_mf = {"lh_MFJ4": 15}
# lateral ff ext side
l_ext_ff = {"lh_FFJ4": 15}
# lateral all int side
l_int_all = {"lh_FFJ4": -15, "lh_MFJ4": -15, "lh_RFJ4": 15, "lh_LFJ4": 15}
# lateral all ext side
l_ext_all = {"lh_FFJ4": 15, "lh_MFJ4": 15, "lh_RFJ4": -15, "lh_LFJ4": -15}
# lateral ff int side
l_int_ff = {"lh_FFJ4": -15}
# lateral mf int side
l_int_mf = {"lh_MFJ4": -15}
# lateral rf int side
l_int_rf = {"lh_RFJ4": 15}
# lateral lf int side
l_int_lf = {"lh_LFJ4": 15}
# all zero
l_zero_all = {"lh_FFJ4": 0, "lh_MFJ4": 0, "lh_RFJ4": 0, "lh_LFJ4": 0}
# spock
l_spock = {"lh_FFJ4": -20, "lh_MFJ4": -20, "lh_RFJ4": -20, "lh_LFJ4": -20}
# store step 1 PST
store_1_PST = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ3": 0, "lh_THJ4": 60, "lh_THJ5": 0,
               "lh_FFJ1": 90, "lh_FFJ2": 90, "lh_FFJ3": 90, "lh_FFJ4": 0,
               "lh_MFJ1": 90, "lh_MFJ2": 90, "lh_MFJ3": 90, "lh_MFJ4": 0,
               "lh_RFJ1": 90, "lh_RFJ2": 90, "lh_RFJ3": 90, "lh_RFJ4": 0,
               "lh_LFJ1": 90, "lh_LFJ2": 90, "lh_LFJ3": 90, "lh_LFJ4": 0, "lh_LFJ5": 0,
               "lh_WRJ1": 0, "lh_WRJ2": 0}
# store step 2 PST
store_2_PST = {"lh_THJ1": 50, "lh_THJ2": 12, "lh_THJ3": 0, "lh_THJ4": 60, "lh_THJ5": 27,
               "lh_FFJ1": 90, "lh_FFJ2": 90, "lh_FFJ3": 90, "lh_FFJ4": 0,
               "lh_MFJ1": 90, "lh_MFJ2": 90, "lh_MFJ3": 90, "lh_MFJ4": 0,
               "lh_RFJ1": 90, "lh_RFJ2": 90, "lh_RFJ3": 90, "lh_RFJ4": 0,
               "lh_LFJ1": 90, "lh_LFJ2": 90, "lh_LFJ3": 90, "lh_LFJ4": 0, "lh_LFJ5": 0,
               "lh_WRJ1": 0, "lh_WRJ2": 0}
# store step 1 Bio_Tac
store_1_BioTac = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ3": 0, "lh_THJ4": 30, "lh_THJ5": 0,
                  "lh_FFJ1": 90, "lh_FFJ2": 90, "lh_FFJ3": 90, "lh_FFJ4": 0,
                  "lh_MFJ1": 90, "lh_MFJ2": 90, "lh_MFJ3": 90, "lh_MFJ4": 0,
                  "lh_RFJ1": 90, "lh_RFJ2": 90, "lh_RFJ3": 90, "lh_RFJ4": 0,
                  "lh_LFJ1": 90, "lh_LFJ2": 90, "lh_LFJ3": 90, "lh_LFJ4": 0, "lh_LFJ5": 0,
                  "lh_WRJ1": 0, "lh_WRJ2": 0}
# store step 2 Bio_Tac
store_2_BioTac = {"lh_THJ1": 20, "lh_THJ2": 36, "lh_THJ3": 0, "lh_THJ4": 30, "lh_THJ5": -3,
                  "lh_FFJ1": 90, "lh_FFJ2": 90, "lh_FFJ3": 90, "lh_FFJ4": 0,
                  "lh_MFJ1": 90, "lh_MFJ2": 90, "lh_MFJ3": 90, "lh_MFJ4": 0,
                  "lh_RFJ1": 90, "lh_RFJ2": 90, "lh_RFJ3": 90, "lh_RFJ4": 0,
                  "lh_LFJ1": 90, "lh_LFJ2": 90, "lh_LFJ3": 90, "lh_LFJ4": 0, "lh_LFJ5": 0,
                  "lh_WRJ1": 0, "lh_WRJ2": 0}
# store step 3
store_3 = {"lh_THJ1": 0, "lh_THJ2": 0, "lh_THJ3": 0, "lh_THJ4": 65, "lh_THJ5": 0}


for x in range(0, 100):
    print("We're on iteration number %d" % (x))
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(store_3, 1.1, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 1.1, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(flex_lf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(ext_lf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(flex_th_1, 0.7, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(flex_th_2, 0.7, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ext_th_1, 1.5, False, angle_degrees=True)
    rospy.sleep(1.5)
    hand_commander.move_to_joint_value_target_unsafe(ext_th_2, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_lf, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
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
    hand_commander.move_to_joint_value_target_unsafe(l_int_lf, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_zero_all, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_spock, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_zero_all, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(pre_ff_ok, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ff_ok, 0.7, False, angle_degrees=True)
    rospy.sleep(0.9)
    hand_commander.move_to_joint_value_target_unsafe(ff2mf_ok, 0.5, False, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(mf_ok, 0.7, False, angle_degrees=True)
    rospy.sleep(0.9)
    hand_commander.move_to_joint_value_target_unsafe(mf2rf_ok, 0.5, False, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(rf_ok, 0.7, False, angle_degrees=True)
    rospy.sleep(0.9)
    hand_commander.move_to_joint_value_target_unsafe(rf2lf_ok, 0.5, False, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(lf_ok, 0.7, False, angle_degrees=True)
    rospy.sleep(0.9)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_lf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_lf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_lf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_lf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(flex_lf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(ext_lf, 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(pre_ff_ok, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ff_ok, 1.3, False, angle_degrees=True)
    rospy.sleep(1.3)
    hand_commander.move_to_joint_value_target_unsafe(ne_wr, 1.1, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(nw_wr, 1.1, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(sw_wr, 1.1, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(se_wr, 1.1, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(ne_wr, 0.7, False, angle_degrees=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(nw_wr, 0.7, False, angle_degrees=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(sw_wr, 0.7, False, angle_degrees=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(se_wr, 0.7, False, angle_degrees=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(zero_wr, 0.4, False, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 1.5, False, angle_degrees=True)
    rospy.sleep(1.5)
