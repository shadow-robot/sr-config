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

rospy.init_node("right_hand_demo", anonymous=True)

hand_commander = SrHandCommander(name="right_hand")

##########
# RANGES #
##########

inter_time_max = 4.0

# Minimum alllowed range for the joints in this particular script
min_range = {"rh_THJ2": -40, "rh_THJ3": -12, "rh_THJ4": 0, "rh_THJ5": -55,
             "rh_FFJ1": 0, "rh_FFJ2": 20, "rh_FFJ3": 0, "rh_FFJ4": -20,
             "rh_MFJ1": 0, "rh_MFJ2": 20, "rh_MFJ3": 0, "rh_MFJ4": -10,
             "rh_RFJ1": 0, "rh_RFJ2": 20, "rh_RFJ3": 0, "rh_RFJ4": -10,
             "rh_LFJ1": 10, "rh_LFJ2": 10, "rh_LFJ3": 0, "rh_LFJ4": -20, "rh_LFJ5": 0,
             "rh_WRJ1": -20, "rh_WRJ2": -10}

# Maximum alllowed range for the joints in this particular script
max_range = {"rh_THJ2": 20, "rh_THJ3": 12, "rh_THJ4": 70, "rh_THJ5": 0,
             "rh_FFJ1": 20, "rh_FFJ2": 90, "rh_FFJ3": 90, "rh_FFJ4": 0,
             "rh_MFJ1": 20, "rh_MFJ2": 90, "rh_MFJ3": 90, "rh_MFJ4": 0,
             "rh_RFJ1": 20, "rh_RFJ2": 90, "rh_RFJ3": 90, "rh_RFJ4": 0,
             "rh_LFJ1": 20, "rh_LFJ2": 90, "rh_LFJ3": 90, "rh_LFJ4": 0, "rh_LFJ5": 1,
             "rh_WRJ1": 10, "rh_WRJ2": 5}

####################
# POSE DEFINITIONS #
####################

# starting position for the hand 
start_pos = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 0, "rh_THJ5": 0,
             "rh_FFJ1": 0, "rh_FFJ2": 0, "rh_FFJ3": 0, "rh_FFJ4": 0,
             "rh_MFJ1": 0, "rh_MFJ2": 0, "rh_MFJ3": 0, "rh_MFJ4": 0,
             "rh_RFJ1": 0, "rh_RFJ2": 0, "rh_RFJ3": 0, "rh_RFJ4": 0,
             "rh_LFJ1": 0, "rh_LFJ2": 0, "rh_LFJ3": 0, "rh_LFJ4": 0, "rh_LFJ5": 0,
             "rh_WRJ1": 0, "rh_WRJ2": 0}
# Start position for the Hand
pregrasp_pos = {"rh_THJ2": 12, "rh_THJ3": 15, "rh_THJ4": 69, "rh_THJ5": -23,
                "rh_FFJ1": 0, "rh_FFJ2": 40, "rh_FFJ3": 21, "rh_FFJ4": -15,
                "rh_MFJ1": 0, "rh_MFJ2": 40, "rh_MFJ3": 21, "rh_MFJ4": 0,
                "rh_RFJ1": 0, "rh_RFJ2": 40, "rh_RFJ3": 21, "rh_RFJ4": -7,
                "rh_LFJ1": 0, "rh_LFJ2": 40, "rh_LFJ3": 21, "rh_LFJ4": -10, "rh_LFJ5": 0,
                "rh_WRJ1": 0, "rh_WRJ2": 0}
# Close position for the Hand
grasp_pos = {"rh_THJ2": 30, "rh_THJ3": 15, "rh_THJ4": 69, "rh_THJ5": -3,
             "rh_FFJ1": 0, "rh_FFJ2": 77, "rh_FFJ3": 67, "rh_FFJ4": -19,
             "rh_MFJ1": 0, "rh_MFJ2": 82, "rh_MFJ3": 62, "rh_MFJ4": 0,
             "rh_RFJ1": 0, "rh_RFJ2": 89, "rh_RFJ3": 64, "rh_RFJ4": -18,
             "rh_LFJ1": 7, "rh_LFJ2": 90, "rh_LFJ3": 64, "rh_LFJ4": -19, "rh_LFJ5": 0,
             "rh_WRJ1": 0, "rh_WRJ2": 0}
# Random position for the Hand (initialied at 0)
rand_pos = {"rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 0, "rh_THJ5": 0,
            "rh_FFJ1": 0, "rh_FFJ2": 0, "rh_FFJ3": 0, "rh_FFJ4": 0,
            "rh_MFJ1": 0, "rh_MFJ2": 0, "rh_MFJ3": 0, "rh_MFJ4": 0,
            "rh_RFJ1": 0, "rh_RFJ2": 0, "rh_RFJ3": 0, "rh_RFJ4": 0,
            "rh_LFJ1": 0, "rh_LFJ2": 0, "rh_LFJ3": 0, "rh_LFJ4": 0, "rh_LFJ5": 0,
            "rh_WRJ1": 0, "rh_WRJ2": 0}
# flex first finger
flex_ff = {"rh_FFJ1": 90, "rh_FFJ2": 90, "rh_FFJ3": 90, "rh_FFJ4": 0}
# extend first finger
ext_ff = {"rh_FFJ1": 0, "rh_FFJ2": 0, "rh_FFJ3": 0, "rh_FFJ4": 0}
# flex middle finger
flex_mf = {"rh_MFJ1": 90, "rh_MFJ2": 90, "rh_MFJ3": 90, "rh_MFJ4": 0}
# extend middle finger
ext_mf = {"rh_MFJ1": 0, "rh_MFJ2": 0, "rh_MFJ3": 0, "rh_MFJ4": 0}
# flex ring finger
flex_rf = {"rh_RFJ1": 90, "rh_RFJ2": 90, "rh_RFJ3": 90, "rh_RFJ4": 0}
# extend ring finger
ext_rf = {"rh_RFJ1": 0, "rh_RFJ2": 0, "rh_RFJ3": 0, "rh_RFJ4": 0}
# flex little finger
flex_lf = {"rh_LFJ1": 90, "rh_LFJ2": 90, "rh_LFJ3": 90, "rh_LFJ4": 0}
# extend middle finger
ext_lf = {"rh_LFJ1": 0, "rh_LFJ2": 0, "rh_LFJ3": 0, "rh_LFJ4": 0}
# flex thumb step 1
flex_th_1 = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 70, "rh_THJ5": 0}
# flex thumb step 2
flex_th_2 = {"rh_THJ1": 35, "rh_THJ2": 38, "rh_THJ3": 10, "rh_THJ4": 70, "rh_THJ5": 58}
# extend thumb step 1
ext_th_1 = {"rh_THJ1": 10, "rh_THJ2": 20, "rh_THJ3": 5, "rh_THJ4": 35, "rh_THJ5": 25}
# extend thumb step 2
ext_th_2 = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 0, "rh_THJ5": 0}
# zero thumb
zero_th = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 0, "rh_THJ5": 0}
# Pre O.K. with first finger
pre_ff_ok = {"rh_THJ4": 70}
# O.K. with first finger
ff_ok = {"rh_THJ1": 17, "rh_THJ2": 20, "rh_THJ3": 0, "rh_THJ4": 56, "rh_THJ5": 18,
         "rh_FFJ1": 0, "rh_FFJ2": 75, "rh_FFJ3": 52, "rh_FFJ4": -0.2,
         "rh_MFJ1": 0, "rh_MFJ2": 42, "rh_MFJ3": 33, "rh_MFJ4": -3,
         "rh_RFJ1": 0, "rh_RFJ2": 50, "rh_RFJ3": 18, "rh_RFJ4": 0.5,
         "rh_LFJ1": 0, "rh_LFJ2": 30, "rh_LFJ3": 0, "rh_LFJ4": -6, "rh_LFJ5": 7}
# O.K. transition from first finger to middle finger
ff2mf_ok = {"rh_THJ1": 5, "rh_THJ2": 12, "rh_THJ3": 4, "rh_THJ4": 60, "rh_THJ5": 2,
            "rh_FFJ1": 0, "rh_FFJ2": 14, "rh_FFJ3": 7, "rh_FFJ4": -0.4,
            "rh_MFJ1": 0, "rh_MFJ2": 42, "rh_MFJ3": 33, "rh_MFJ4": -3,
            "rh_RFJ1": 0, "rh_RFJ2": 50, "rh_RFJ3": 18, "rh_RFJ4": 0.5,
            "rh_LFJ1": 0, "rh_LFJ2": 30, "rh_LFJ3": 0, "rh_LFJ4": -6, "rh_LFJ5": 7}
# O.K. with middle finger
mf_ok = {"rh_THJ1": 19, "rh_THJ2": 17, "rh_THJ3": 6, "rh_THJ4": 66, "rh_THJ5": 31,
         "rh_FFJ1": 0, "rh_FFJ2": 15, "rh_FFJ3": 7, "rh_FFJ4": -0.4,
         "rh_MFJ1": 11, "rh_MFJ2": 71, "rh_MFJ3": 49, "rh_MFJ4": 10,
         "rh_RFJ1": 0, "rh_RFJ2": 50, "rh_RFJ3": 18, "rh_RFJ4": -10,
         "rh_LFJ1": 0, "rh_LFJ2": 30, "rh_LFJ3": 0, "rh_LFJ4": -6, "rh_LFJ5": 7}
# O.K. transition from middle finger to ring finger
mf2rf_ok = {"rh_THJ1": 5, "rh_THJ2": -5, "rh_THJ3": 15, "rh_THJ4": 70, "rh_THJ5": 19,
            "rh_FFJ1": 0, "rh_FFJ2": 14, "rh_FFJ3": 7, "rh_FFJ4": -0.4,
            "rh_MFJ1": 0, "rh_MFJ2": 45, "rh_MFJ3": 4, "rh_MFJ4": -1,
            "rh_RFJ1": 0, "rh_RFJ2": 50, "rh_RFJ3": 18, "rh_RFJ4": -19,
            "rh_LFJ1": 0, "rh_LFJ2": 30, "rh_LFJ3": 0, "rh_LFJ4": -12, "rh_LFJ5": 7}
# O.K. with ring finger
rf_ok = {"rh_THJ1": 8, "rh_THJ2": 15, "rh_THJ3": 15, "rh_THJ4": 70, "rh_THJ5": 45,
         "rh_FFJ1": 0, "rh_FFJ2": 14, "rh_FFJ3": 7, "rh_FFJ4": -0.4,
         "rh_MFJ1": 0, "rh_MFJ2": 45, "rh_MFJ3": 4, "rh_MFJ4": -1,
         "rh_RFJ1": 3, "rh_RFJ2": 90, "rh_RFJ3": 42, "rh_RFJ4": -19,
         "rh_LFJ1": 0, "rh_LFJ2": 30, "rh_LFJ3": 0, "rh_LFJ4": -12, "rh_LFJ5": 7}
# O.K. transition from ring finger to little finger
rf2lf_ok = {"rh_THJ1": 5, "rh_THJ2": 4.5, "rh_THJ3": 8, "rh_THJ4": 60, "rh_THJ5": 21,
            "rh_FFJ1": 0, "rh_FFJ2": 14, "rh_FFJ3": 7, "rh_FFJ4": -0.4,
            "rh_MFJ1": 0, "rh_MFJ2": 45, "rh_MFJ3": 4, "rh_MFJ4": -1,
            "rh_RFJ1": 0, "rh_RFJ2": 30, "rh_RFJ3": 6, "rh_RFJ4": 0.5,
            "rh_LFJ1": 0, "rh_LFJ2": 30, "rh_LFJ3": 0, "rh_LFJ4": -10, "rh_LFJ5": 7}
# O.K. with little finger
lf_ok = {"rh_THJ1": 25, "rh_THJ2": 14, "rh_THJ3": 10, "rh_THJ4": 69, "rh_THJ5": 22,
         "rh_FFJ1": 0, "rh_FFJ2": 14, "rh_FFJ3": 7, "rh_FFJ4": -0.4,
         "rh_MFJ1": 0, "rh_MFJ2": 15, "rh_MFJ3": 4, "rh_MFJ4": -1,
         "rh_RFJ1": 0, "rh_RFJ2": 15, "rh_RFJ3": 6, "rh_RFJ4": 0.5,
         "rh_LFJ1": 0, "rh_LFJ2": 78, "rh_LFJ3": 26, "rh_LFJ4": 5, "rh_LFJ5": 37}
# zero wrist
zero_wr = {"rh_WRJ1": 0, "rh_WRJ2": 0}
# north wrist
n_wr = {"rh_WRJ1": 15, "rh_WRJ2": 0}
# south wrist
s_wr = {"rh_WRJ1": -20, "rh_WRJ2": 0}
# east wrist
e_wr = {"rh_WRJ1": 0, "rh_WRJ2": 8}
# west wrist
w_wr = {"rh_WRJ1": 0, "rh_WRJ2": -14}
# northeast wrist
ne_wr = {"rh_WRJ1": 15, "rh_WRJ2": 8}
# northwest wrist
nw_wr = {"rh_WRJ1": 15, "rh_WRJ2": -14}
# southweast wrist
sw_wr = {"rh_WRJ1": -20, "rh_WRJ2": -14}
# southeast wrist
se_wr = {"rh_WRJ1": -20, "rh_WRJ2": 8}
# lateral lf ext side
l_ext_lf = {"rh_LFJ4": -15}
# lateral rf ext side
l_ext_rf = {"rh_RFJ4": -15}
# lateral mf ext side
l_ext_mf = {"rh_MFJ4": 15}
# lateral ff ext side
l_ext_ff = {"rh_FFJ4": 15}
# lateral all int side
l_int_all = {"rh_FFJ4": -15, "rh_MFJ4": -15, "rh_RFJ4": 15, "rh_LFJ4": 15}
# lateral all ext side
l_ext_all = {"rh_FFJ4": 15, "rh_MFJ4": 15, "rh_RFJ4": -15, "rh_LFJ4": -15}
# lateral ff int side
l_int_ff = {"rh_FFJ4": -15}
# lateral mf int side
l_int_mf = {"rh_MFJ4": -15}
# lateral rf int side
l_int_rf = {"rh_RFJ4": 15}
# lateral lf int side
l_int_lf = {"rh_LFJ4": 15}
# all zero
l_zero_all = {"rh_FFJ4": 0, "rh_MFJ4": 0, "rh_RFJ4": 0, "rh_LFJ4": 0}
# spock
l_spock = {"rh_FFJ4": -20, "rh_MFJ4": -20, "rh_RFJ4": -20, "rh_LFJ4": -20}
# grasp for shaking hands step 1
shake_grasp_1 = {"rh_THJ1": 0, "rh_THJ2": 6, "rh_THJ3": 10, "rh_THJ4": 37, "rh_THJ5": 9,
                 "rh_FFJ1": 0, "rh_FFJ2": 21, "rh_FFJ3": 26, "rh_FFJ4": 0,
                 "rh_MFJ1": 0, "rh_MFJ2": 18, "rh_MFJ3": 24, "rh_MFJ4": 0,
                 "rh_RFJ1": 0, "rh_RFJ2": 30, "rh_RFJ3": 16, "rh_RFJ4": 0,
                 "rh_LFJ1": 0, "rh_LFJ2": 30, "rh_LFJ3": 0, "rh_LFJ4": -10, "rh_LFJ5": 10}
# grasp for shaking hands step 2
shake_grasp_2 = {"rh_THJ1": 21, "rh_THJ2": 21, "rh_THJ3": 10, "rh_THJ4": 42, "rh_THJ5": 21,
                 "rh_FFJ1": 0, "rh_FFJ2": 75, "rh_FFJ3": 29, "rh_FFJ4": 0,
                 "rh_MFJ1": 0, "rh_MFJ2": 75, "rh_MFJ3": 41, "rh_MFJ4": 0,
                 "rh_RFJ1": 0, "rh_RFJ2": 75, "rh_RFJ3": 41, "rh_RFJ4": 0,
                 "rh_LFJ1": 10, "rh_LFJ2": 90, "rh_LFJ3": 41, "rh_LFJ4": 0, "rh_LFJ5": 0}
# store step 1 PST
store_1_PST = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 60, "rh_THJ5": 0,
               "rh_FFJ1": 90, "rh_FFJ2": 90, "rh_FFJ3": 90, "rh_FFJ4": 0,
               "rh_MFJ1": 90, "rh_MFJ2": 90, "rh_MFJ3": 90, "rh_MFJ4": 0,
               "rh_RFJ1": 90, "rh_RFJ2": 90, "rh_RFJ3": 90, "rh_RFJ4": 0,
               "rh_LFJ1": 90, "rh_LFJ2": 90, "rh_LFJ3": 90, "rh_LFJ4": 0, "rh_LFJ5": 0,
               "rh_WRJ1": 0, "rh_WRJ2": 0}
# store step 2 PST
store_2_PST = {"rh_THJ1": 50, "rh_THJ2": 12, "rh_THJ3": 0, "rh_THJ4": 60, "rh_THJ5": 27,
               "rh_FFJ1": 90, "rh_FFJ2": 90, "rh_FFJ3": 90, "rh_FFJ4": 0,
               "rh_MFJ1": 90, "rh_MFJ2": 90, "rh_MFJ3": 90, "rh_MFJ4": 0,
               "rh_RFJ1": 90, "rh_RFJ2": 90, "rh_RFJ3": 90, "rh_RFJ4": 0,
               "rh_LFJ1": 90, "rh_LFJ2": 90, "rh_LFJ3": 90, "rh_LFJ4": 0, "rh_LFJ5": 0,
               "rh_WRJ1": 0, "rh_WRJ2": 0}
# store step 1 Bio_Tac
store_1_BioTac = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 30, "rh_THJ5": 0,
                  "rh_FFJ1": 90, "rh_FFJ2": 90, "rh_FFJ3": 90, "rh_FFJ4": 0,
                  "rh_MFJ1": 90, "rh_MFJ2": 90, "rh_MFJ3": 90, "rh_MFJ4": 0,
                  "rh_RFJ1": 90, "rh_RFJ2": 90, "rh_RFJ3": 90, "rh_RFJ4": 0,
                  "rh_LFJ1": 90, "rh_LFJ2": 90, "rh_LFJ3": 90, "rh_LFJ4": 0, "rh_LFJ5": 0,
                  "rh_WRJ1": 0, "rh_WRJ2": 0}
# store step 2 Bio_Tac
store_2_BioTac = {"rh_THJ1": 20, "rh_THJ2": 36, "rh_THJ3": 0, "rh_THJ4": 30, "rh_THJ5": -3,
                  "rh_FFJ1": 90, "rh_FFJ2": 90, "rh_FFJ3": 90, "rh_FFJ4": 0,
                  "rh_MFJ1": 90, "rh_MFJ2": 90, "rh_MFJ3": 90, "rh_MFJ4": 0,
                  "rh_RFJ1": 90, "rh_RFJ2": 90, "rh_RFJ3": 90, "rh_RFJ4": 0,
                  "rh_LFJ1": 90, "rh_LFJ2": 90, "rh_LFJ3": 90, "rh_LFJ4": 0, "rh_LFJ5": 0,
                  "rh_WRJ1": 0, "rh_WRJ2": 0}
# store step 3
store_3 = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 65, "rh_THJ5": 0}
# business card pre-zero position 
bc_pre_zero = {"rh_THJ1": 15, "rh_THJ2": 7, "rh_THJ3": -4, "rh_THJ4": 41, "rh_THJ5": -20,
               "rh_FFJ1": 0, "rh_FFJ2": 14, "rh_FFJ3": 7, "rh_FFJ4": -1,
               "rh_MFJ1": 0, "rh_MFJ2": 51, "rh_MFJ3": 33, "rh_MFJ4": 20,
               "rh_RFJ1": 0, "rh_RFJ2": 50, "rh_RFJ3": 18, "rh_RFJ4": -20,
               "rh_LFJ1": 0, "rh_LFJ2": 30, "rh_LFJ3": 0, "rh_LFJ4": -20, "rh_LFJ5": 7}
# business card zero position 
bc_zero = {"rh_THJ1": 38, "rh_THJ2": 4, "rh_THJ3": 0, "rh_THJ4": 48, "rh_THJ5": -5,
           "rh_MFJ1": 7, "rh_MFJ2": 64, "rh_MFJ3": 20, "rh_MFJ4": 18}
# business card position 1 
bc_1 = {"rh_FFJ1": 47, "rh_FFJ2": 90, "rh_FFJ3": 7}
# business card position 2 
bc_2 = {"rh_FFJ1": 47, "rh_FFJ2": 90, "rh_FFJ3": 58}
# business card position 3 
bc_3 = {"rh_FFJ1": 0, "rh_FFJ2": 60, "rh_FFJ3": 58}
# business card position 4 
bc_4 = {"rh_FFJ1": 90, "rh_FFJ2": 90, "rh_FFJ3": 58, "rh_FFJ4": 15}
# business card position 5 
bc_5 = {"rh_FFJ1": 90, "rh_FFJ2": 90, "rh_FFJ3": 0}
# business card position 6 
bc_6 = {"rh_FFJ1": 0, "rh_FFJ2": 0, "rh_FFJ3": 0, "rh_FFJ4": 10}
# business card position 7 
bc_7 = {"rh_FFJ1": 47, "rh_FFJ2": 90, "rh_FFJ3": 15, "rh_FFJ4": 0}
# business card position 8 
bc_8 = {"rh_FFJ1": 47, "rh_FFJ2": 90, "rh_FFJ3": 58}
# business card position 9 
bc_9 = {"rh_FFJ1": 0, "rh_FFJ2": 71, "rh_FFJ3": 58}
# business card position 10 
bc_10 = {"rh_MFJ3": 64, "rh_FFJ4": 20}
# business card position 11 
bc_11 = {"rh_FFJ1": 0, "rh_FFJ2": 81, "rh_FFJ3": 50, "rh_FFJ4": 20,
         "rh_THJ4": 57, "rh_THJ5": 20,}
# business card position 12 
bc_12 = {"rh_MFJ1": 0, "rh_MFJ2": 20, "rh_MFJ3": 10, "rh_MFJ4": 0}


########################
# FUNCTION DEFINITIONS #
########################

def secuence_ff():
    # Start secuence 1
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(store_3, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 1.2, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, 1.0, False, angle_degrees=True)
    rospy.sleep(1.3)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.3)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.3)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(flex_lf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.3)
    hand_commander.move_to_joint_value_target_unsafe(ext_lf, 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(flex_th_1, 0.7, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(flex_th_2, 0.7, False, angle_degrees=True)
    rospy.sleep(1)
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
    hand_commander.move_to_joint_value_target_unsafe(ff_ok, 1.0, False, angle_degrees=True)
    rospy.sleep(2)
    hand_commander.move_to_joint_value_target_unsafe(ff2mf_ok, 0.8, False, angle_degrees=True)
    rospy.sleep(1.0)
    hand_commander.move_to_joint_value_target_unsafe(mf_ok, 1.0, False, angle_degrees=True)
    rospy.sleep(2)
    hand_commander.move_to_joint_value_target_unsafe(mf2rf_ok, 0.8, False, angle_degrees=True)
    rospy.sleep(1.0)
    hand_commander.move_to_joint_value_target_unsafe(rf_ok, 1.0, False, angle_degrees=True)
    rospy.sleep(2)
    hand_commander.move_to_joint_value_target_unsafe(rf2lf_ok, 0.8, False, angle_degrees=True)
    rospy.sleep(1.0)
    hand_commander.move_to_joint_value_target_unsafe(lf_ok, 1.0, False, angle_degrees=True)
    rospy.sleep(2)
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
    hand_commander.move_to_joint_value_target_unsafe(ff_ok, 2.0, False, angle_degrees=True)
    rospy.sleep(2)
    hand_commander.move_to_joint_value_target_unsafe(ne_wr, 1.4, False, angle_degrees=True)
    rospy.sleep(1.4)
    hand_commander.move_to_joint_value_target_unsafe(nw_wr, 1.4, False, angle_degrees=True)
    rospy.sleep(1.4)
    hand_commander.move_to_joint_value_target_unsafe(sw_wr, 1.4, False, angle_degrees=True)
    rospy.sleep(1.4)
    hand_commander.move_to_joint_value_target_unsafe(se_wr, 1.4, False, angle_degrees=True)
    rospy.sleep(1.4)
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
    return


def secuence_mf():
    # Start the secuence 2
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

            hand_commander.move_to_joint_value_target_unsafe(start_pos, 2.0, False, angle_degrees=True)
            print 'HAND TOUCHED!'
            rospy.sleep(2.0)

            if (tactile_values['TH'] > force_zero['TH']):
                break

        # If the tactile sensors have not been triggered and the Hand
        # is not in the middle of a movement, generate a random position
        # and interpolation time
        else:
            if time.time() > wake_time:
                for i in rand_pos:
                    rand_pos[i] = random.randrange(min_range[i], max_range[i])

                rand_pos['rh_FFJ4'] = random.randrange(min_range['rh_FFJ4'], rand_pos['rh_MFJ4'])
                rand_pos['rh_LFJ4'] = random.randrange(min_range['rh_LFJ4'], rand_pos['rh_RFJ4'])
                inter_time = inter_time_max * random.random()
                #            rand_pos['interpolation_time'] = max_range['interpolation_time'] * random.random()

                hand_commander.move_to_joint_value_target_unsafe(rand_pos, inter_time, False, angle_degrees=True)
                wake_time = time.time() + inter_time * 0.9
    return


def secuence_rf():
    # Start the secuence 3
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(bc_pre_zero, 2.0, False, angle_degrees=True)
    rospy.sleep(2)
    hand_commander.move_to_joint_value_target_unsafe(bc_zero, 1.0, False, angle_degrees=True)
    rospy.sleep(4)
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
    hand_commander.move_to_joint_value_target_unsafe(bc_11, 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(bc_12, 3.0, False, angle_degrees=True)
    rospy.sleep(4)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, 1.5, False, angle_degrees=True)
    rospy.sleep(1.5)

    return


def secuence_lf():
    # Start the secuence 4
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
        # Check  the state of the tactile senors
        read_tactile_values()

        # Record current joint positions
        hand_pos = hand_commander.get_joints_position()

        # If any tacticle sensor has been triggered, send
        # the corresponding digit to its current position
        if (tactile_values['FF'] > force_zero['FF'] and trigger[0] == 0):
            hand_pos_incr_f = {"rh_FFJ1": hand_pos['rh_FFJ1'] + offset1, "rh_FFJ3": hand_pos['rh_FFJ3'] + offset1}
            hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_f, 0.5, False, angle_degrees=True)
            print 'First finger contact'
            trigger[0] = 1

        if (tactile_values['MF'] > force_zero['MF'] and trigger[1] == 0):
            hand_pos_incr_m = {"rh_MFJ1": hand_pos['rh_MFJ1'] + offset1, "rh_MFJ3": hand_pos['rh_MFJ3'] + offset1}
            hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_m, 0.5, False, angle_degrees=True)
            print 'Middle finger contact'
            trigger[1] = 1

        if (tactile_values['RF'] > force_zero['RF'] and trigger[2] == 0):
            hand_pos_incr_r = {"rh_RFJ1": hand_pos['rh_RFJ1'] + offset1, "rh_RFJ3": hand_pos['rh_RFJ3'] + offset1}
            hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_r, 0.5, False, angle_degrees=True)
            print 'Ring finger contact'
            trigger[2] = 1

        if (tactile_values['LF'] > force_zero['LF'] and trigger[3] == 0):
            hand_pos_incr_l = {"rh_LFJ1": hand_pos['rh_LFJ1'] + offset1, "rh_LFJ3": hand_pos['rh_LFJ3'] + offset1}
            hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_l, 0.5, False, angle_degrees=True)
            print 'Little finger contact'
            trigger[3] = 1

        if (tactile_values['TH'] > force_zero['TH'] and trigger[4] == 0):
            hand_pos_incr_th = {"rh_THJ2": hand_pos['rh_THJ2'] + offset1, "rh_THJ5": hand_pos['rh_THJ5'] + offset1}
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
    squeeze.update({"rh_THJ5": hand_pos['rh_THJ5'] + offset2, "rh_THJ2": hand_pos['rh_THJ2'] + offset2,
                    "rh_FFJ3": hand_pos['rh_FFJ3'] + offset2, "rh_FFJ1": hand_pos['rh_FFJ1'] + offset2,
                    "rh_MFJ3": hand_pos['rh_MFJ3'] + offset2, "rh_MFJ1": hand_pos['rh_MFJ1'] + offset2,
                    "rh_RFJ3": hand_pos['rh_RFJ3'] + offset2, "rh_RFJ1": hand_pos['rh_RFJ1'] + offset2,
                    "rh_LFJ3": hand_pos['rh_LFJ3'] + offset2, "rh_LFJ1": hand_pos['rh_LFJ1'] + offset2})

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


def secuence_th():
    # Start the secuence 5
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

    force_zero['FF'] = force_zero['FF'] + 5
    force_zero['MF'] = force_zero['MF'] + 5
    force_zero['RF'] = force_zero['RF'] + 5
    force_zero['LF'] = force_zero['LF'] + 5
    force_zero['TH'] = force_zero['TH'] + 5

    print 'Force Zero', force_zero

    rospy.loginfo("\n\nOK, ready for the demo")

    print "\nPRESS ONE OF THE TACTILES TO START A DEMO"
    print "   FF: Standard Demo"
    print "   MF: Shy Hand Demo"
    print "   RF: Card Trick Demo"
    print "   LF: Grasp Demo"
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

    elif tactile_type == None:
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
    if (tactile_values['FF'] > force_zero['FF']):
        print 'First finger contact'
        secuence_ff()
        print 'FF demo completed'
        zero_tactile_sensors()
    if (tactile_values['MF'] > force_zero['MF']):
        print 'Middle finger contact'
        secuence_mf()
        print 'MF demo completed'
        zero_tactile_sensors()
    if (tactile_values['RF'] > force_zero['RF']):
        print 'Ring finger contact'
        secuence_rf()
        print 'RF demo completed'
        zero_tactile_sensors()
    if (tactile_values['LF'] > force_zero['LF']):
        print 'Little finger contact'
        secuence_lf()
        print 'LF demo completed'
        zero_tactile_sensors()
    if (tactile_values['TH'] > force_zero['TH']):
        print 'Thumb finger contact'
        secuence_th()
        print 'TH demo completed'
        zero_tactile_sensors()



