#!/usr/bin/env python

#
# Copyright 2019 Shadow Robot Company Ltd.
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

# Script to move the left hand into store position.

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("store_left_hand", anonymous=True)

hand_commander = SrHandCommander(name="left_hand")

open_hand = {'lh_FFJ1': 0.0, 'lh_FFJ2': 0.0, 'lh_FFJ3': 0.0, 'lh_FFJ4': 0.0,
             'lh_MFJ1': 0.0, 'lh_MFJ2': 0.0, 'lh_MFJ3': 0.0, 'lh_MFJ4': 0.0,
             'lh_RFJ1': 0.0, 'lh_RFJ2': 0.0, 'lh_RFJ3': 0.0, 'lh_RFJ4': 0.0,
             'lh_LFJ1': 0.0, 'lh_LFJ2': 0.0, 'lh_LFJ3': 0.0, 'lh_LFJ4': 0.0, 'lh_LFJ5': 0.0,
             'lh_THJ1': 0.0, 'lh_THJ2': 0.0, 'lh_THJ3': 0.0, 'lh_THJ4': 0.0, 'lh_THJ5': 0.0,
             'lh_WRJ1': 0.0, 'lh_WRJ2': 0.0}

pack_hand_1 = {'lh_FFJ1': 1.5707, 'lh_FFJ2': 1.5707, 'lh_FFJ3': 1.5707, 'lh_FFJ4': 0.0,
               'lh_MFJ1': 1.5707, 'lh_MFJ2': 1.5707, 'lh_MFJ3': 1.5707, 'lh_MFJ4': 0.0,
               'lh_RFJ1': 1.5707, 'lh_RFJ2': 1.5707, 'lh_RFJ3': 1.5707, 'lh_RFJ4': 0.0,
               'lh_LFJ1': 1.5707, 'lh_LFJ2': 1.5707, 'lh_LFJ3': 1.5707, 'lh_LFJ4': 0.0, 'lh_LFJ5': 0.0}

pack_hand_2 = {'lh_THJ4': 1.2}

pack_hand_3 = {'lh_THJ1': 0.87, 'lh_THJ2': 0.49, 'lh_THJ5': 0.43}


# Move hand to open position
joint_states = open_hand
rospy.loginfo("Moving hand to open position")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, False)
rospy.sleep(2)

# Move hand to closed position
joint_states = pack_hand_1
rospy.loginfo("Moving hand to pack position")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, False)
rospy.sleep(2)

joint_states = pack_hand_2
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, False)
rospy.sleep(2)

joint_states = pack_hand_3
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, False)
rospy.sleep(2)
