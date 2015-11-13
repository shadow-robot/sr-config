#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
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

import roslib; roslib.load_manifest('sr_hand')
import rospy

import time, mutex, subprocess, math
#!/usr/bin/env python

# Script to move the left hand into store position.

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("right_hand_demo", anonymous=True)

hand_commander = SrHandCommander(name="right_hand")

open_hand = {'lh_FFJ1': 0.0, 'lh_FFJ2': 0.0, 'lh_FFJ3': 0.0, 'lh_FFJ4': 0.0,
             'lh_MFJ1': 0.0, 'lh_MFJ2': 0.0, 'lh_MFJ3': 0.0, 'lh_MFJ4': 0.0,
             'lh_RFJ1': 0.0, 'lh_RFJ2': 0.0, 'lh_RFJ3': 0.0, 'lh_RFJ4': 0.0,
             'lh_LFJ1': 0.0, 'lh_LFJ2': 0.0, 'lh_LFJ3': 0.0, 'lh_LFJ4': 0.0, 'lh_LFJ5': 0.0,
             'lh_THJ1': 0.0, 'lh_THJ2': 0.0, 'lh_THJ3': 0.0, 'lh_THJ4': 0.0, 'lh_THJ5': 0.0,
             'lh_WRJ1': 0.0, 'lh_WRJ2': 0.0}

natural_hand = {'lh_FFJ1': -0.015231161684773453, 'lh_FFJ2': 0.4707354376773168, 'lh_FFJ3': -0.020599036757731948, 'lh_FFJ4': 0.03846792953124516, 'lh_THJ4': 1.129553058484541, 'lh_THJ5': 0.17285609095207696, 'lh_THJ1': 0.270633081305584, 'lh_THJ2': 0.7282881900694186, 'lh_THJ3': -0.015406461537391941, 'lh_LFJ2': 0.5850595799574672, 'lh_LFJ3': 0.5636517849387783, 'lh_LFJ1': 0.03461076652259942, 'lh_LFJ4': -0.28529553092230801, 'lh_LFJ5': 0.14943387379671536, 'lh_RFJ4': -0.030063596923893404, 'lh_RFJ1': 0.005949986086344283, 'lh_RFJ2': 0.6274874215503423, 'lh_RFJ3': 0.44395101691519906, 'lh_MFJ1': 0.025355373778972734, 'lh_MFJ3': 0.07746660184405069, 'lh_MFJ2': 0.9605842806822085, 'lh_MFJ4': 0.02818898576846175, 'lh_WRJ2': -0.05968479660514132, 'lh_WRJ1': -0.016812134199197103}

FF_Ext = {'lh_FFJ1': 0.0, 'lh_FFJ2': 0.0, 'lh_FFJ3': 0.0}
FF_Flex = {'lh_FFJ1': 1.5707, 'lh_FFJ2': 1.5707, 'lh_FFJ3': 1.5707}
MF_Ext = {'lh_MFJ1': 0.0, 'lh_MFJ2': 0.0, 'lh_MFJ3': 0.0}
MF_Flex = {'lh_MFJ1': 1.5707, 'lh_MFJ2': 1.5707, 'lh_MFJ3': 1.5707}
RF_Ext = {'lh_RFJ1': 0.0, 'lh_RFJ2': 0.0, 'lh_RFJ3': 0.0}
RF_Flex = {'lh_RFJ1': 1.5707, 'lh_RFJ2': 1.5707, 'lh_RFJ3': 1.5707}
LF_Ext = {'lh_LFJ1': 0.0, 'lh_LFJ2': 0.0, 'lh_LFJ3': 0.0}
LF_Flex = {'lh_LFJ1': 1.5707, 'lh_LFJ2': 1.5707, 'lh_LFJ3': 1.5707}
TH_Ext = {'lh_THJ1': 0.0, 'lh_THJ2': 0.0, 'lh_THJ3': 0.0, 'lh_THJ4': 0.0, 'lh_THJ5': 0.0}
TH_Flex = {'lh_THJ1': 0.7854, 'lh_THJ2': 0.262, 'lh_THJ3': 0.209, 'lh_THJ4': 1.222, 'lh_THJ5': 1.00}
WR1_Ext = {'lh_WRJ1': -0.349}
WR1_Flex = {'lh_WRJ1': 0.349}
WR2_Ext = {'lh_WRJ2': -0.349}
WR2_Flex = {'lh_WRJ2': 0.140}
WR_Zero = {'lh_WRJ1': 0.0, 'lh_WRJ2': 0.0}

AbdAdd_0 = {'lh_FFJ4': 0.0, 'lh_MFJ4': 0.0, 'lh_RFJ4': 0.0, 'lh_LFJ4': 0.0}
AbdAdd_1 = {'lh_FFJ4': 0.0, 'lh_MFJ4': 0.0, 'lh_RFJ4': 0.0, 'lh_LFJ4': -0.349}
AbdAdd_2 = {'lh_FFJ4': 0.0, 'lh_MFJ4': 0.0, 'lh_RFJ4': -0.349, 'lh_LFJ4': -0.349}
AbdAdd_3 = {'lh_FFJ4': 0.0, 'lh_MFJ4': 0.349, 'lh_RFJ4': -0.349, 'lh_LFJ4': -0.349}
AbdAdd_4 = {'lh_FFJ4': 0.349, 'lh_MFJ4': 0.349, 'lh_RFJ4': -0.349, 'lh_LFJ4': -0.349}
AbdAdd_5 = {'lh_FFJ4': -0.349, 'lh_MFJ4': -0.349, 'lh_RFJ4': 0.349, 'lh_LFJ4': 0.349}
AbdAdd_6 = {'lh_FFJ4': -0.349, 'lh_MFJ4': 0.349, 'lh_RFJ4': -0.349, 'lh_LFJ4': -0.349}
AbdAdd_7 = {'lh_FFJ4': -0.349, 'lh_MFJ4': -0.349, 'lh_RFJ4': -0.349, 'lh_LFJ4': -0.349}
AbdAdd_8 = {'lh_FFJ4': -0.349, 'lh_MFJ4': -0.349, 'lh_RFJ4': 0.349, 'lh_LFJ4': -0.349}
AbdAdd_9 = {'lh_FFJ4': -0.349, 'lh_MFJ4': -0.349, 'lh_RFJ4': 0.349, 'lh_LFJ4': 0.349}

FF_OK = {'lh_FFJ1': 0.022515630316621627, 'lh_FFJ2': 1.2556036427998678, 'lh_FFJ3': 0.8447767823310574, 'lh_FFJ4': 0.04590339173466104, 'lh_THJ4': 1.0949407683205896, 'lh_THJ5': 0.14200564847861585, 'lh_THJ1': 0.3683921782143842, 'lh_THJ2': 0.6373691082133975, 'lh_THJ3': -0.0244610170861788, 'lh_LFJ2': 0.5841731260484406, 'lh_LFJ3': 0.5632873801031884, 'lh_LFJ1': 0.03461076652259942, 'lh_LFJ4': -0.28125963828545088, 'lh_LFJ5': 0.1443737718056313, 'lh_RFJ4': -0.02468880994975094, 'lh_RFJ1': 0.0047599888690754155, 'lh_RFJ2': 0.6264485350908219, 'lh_RFJ3': 0.44382005595085083, 'lh_MFJ1': 0.021644831274732823, 'lh_MFJ3': 0.06212910667048069, 'lh_MFJ2': 0.9476510773927296, 'lh_MFJ4': 0.024157387999199982, 'lh_WRJ2': -0.05952234866755241, 'lh_WRJ1': -0.017882833898283462}

MF_OK = {'lh_FFJ1': 0.01192003957938792, 'lh_FFJ2': 0.8779137365294785, 'lh_FFJ3': -0.04879151632568085, 'lh_FFJ4': 0.03850631221477557, 'lh_THJ4': 1.1353427378511094, 'lh_THJ5': 0.25622286887027657, 'lh_THJ1': 0.41978526202199823, 'lh_THJ2': 0.6871574647024502, 'lh_THJ3': 0.052341509424183565, 'lh_LFJ2': 0.5832866721394142, 'lh_LFJ3': 0.5626116821752853, 'lh_LFJ1': 0.0339451748587033, 'lh_LFJ4': -0.28131817973754372, 'lh_LFJ5': 0.14504552420005684, 'lh_RFJ4': -0.023773075709565508, 'lh_RFJ1': 0.004164990260441037, 'lh_RFJ2': 0.6212541027932197, 'lh_RFJ3': 0.4446185880909443, 'lh_MFJ1': 0.15027697142171648, 'lh_MFJ3': 0.8577206183860272, 'lh_MFJ2': 1.2647385897689567, 'lh_MFJ4': 0.007472558773148949, 'lh_WRJ2': -0.05935494962092569, 'lh_WRJ1': -0.00794173869246716}

RF_OK = {'lh_FFJ1': 0.012582264000465027, 'lh_FFJ2': 0.892676859901611, 'lh_FFJ3': -0.01668023341905365, 'lh_FFJ4': 0.03684708613889836, 'lh_THJ4': 1.3244270470831758, 'lh_THJ5': 0.5461264574235202, 'lh_THJ1': 0.04931913989452231, 'lh_THJ2': 0.7723593776592549, 'lh_THJ3': 0.2332251218233079, 'lh_LFJ2': 0.5380775227790644, 'lh_LFJ3': 0.5378242421019102, 'lh_LFJ1': 0.029286033211430307, 'lh_LFJ4': -0.17266088110429886, 'lh_LFJ5': 0.11609954153828721, 'lh_RFJ4': -0.1410235306590761, 'lh_RFJ1': 0.04343489843031345, 'lh_RFJ2': 1.4093381677689403, 'lh_RFJ3': 0.7687871061350975, 'lh_MFJ1': 0.03710542504239911, 'lh_MFJ3': 0.11668324341742378, 'lh_MFJ2': 0.9358936198568396, 'lh_MFJ4': 0.03313362638105717, 'lh_WRJ2': -0.06225971637785656, 'lh_WRJ1': -0.009767850763861427}

LF_OK = {'lh_FFJ1': 0.00397334652646264, 'lh_FFJ2': 0.8975979010256552, 'lh_FFJ3': -0.0016476274144017463, 'lh_FFJ4': 0.03250985089166586, 'lh_THJ4': 1.2267774543912607, 'lh_THJ5': 0.20310694069887214, 'lh_THJ1': 0.2888424727858525, 'lh_THJ2': 0.5641506091726277, 'lh_THJ3': 0.23278402932191808, 'lh_LFJ2': 1.4468938248187885, 'lh_LFJ3': 0.2733478661324566, 'lh_LFJ1': 0.01406673264624339, 'lh_LFJ4': -0.22573803931632768, 'lh_LFJ5': 0.7547867069581923, 'lh_RFJ4': -0.1442490766383741, 'lh_RFJ1': 0.03331992208352813, 'lh_RFJ2': 1.2615629035756921, 'lh_RFJ3': 0.158866717571428, 'lh_MFJ1': 0.03215803503674591, 'lh_MFJ3': 0.14292484557904367, 'lh_MFJ2': 0.9464753316391405, 'lh_MFJ4': 0.023358296483200973, 'lh_WRJ2': -0.05471159545568462, 'lh_WRJ1': -0.011152559328240493}


hand_commander.move_to_joint_value_target_unsafe(open_hand, 2.0, True)
rospy.sleep(2)
hand_commander.move_to_joint_value_target_unsafe(FF_Flex, 0.75, True)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(FF_Ext, 0.75, True)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(MF_Flex, 0.75, True)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(MF_Ext, 0.75, True)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(RF_Flex, 0.75, True)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(RF_Ext, 0.75, True)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(LF_Flex, 0.75, True)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(LF_Ext, 0.75, True)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(TH_Flex, 1.0, True)
rospy.sleep(1)
hand_commander.move_to_joint_value_target_unsafe(TH_Ext, 1.0, True)
rospy.sleep(1)
hand_commander.move_to_joint_value_target_unsafe(WR1_Flex, 0.75, True)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(WR1_Ext, 0.75, True)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(WR2_Flex, 0.75, True)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(WR2_Ext, 0.75, True)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(WR_Zero, 0.75, True)
rospy.sleep(1)

hand_commander.move_to_joint_value_target_unsafe(AbdAdd_0, 0.25, True)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_1, 0.25, True)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_2, 0.25, True)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_3, 0.25, True)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_4, 0.25, True)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_5, 0.25, True)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_4, 0.25, True)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_6, 0.25, True)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_7, 0.25, True)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_8, 0.25, True)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_9, 0.25, True)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_0, 0.25, True)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_7, 0.25, True)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_0, 0.25, True)
rospy.sleep(0.25)

hand_commander.move_to_joint_value_target_unsafe(natural_hand, 0.5, True)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(FF_OK, 0.5, True)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(natural_hand, 0.5, True)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(MF_OK, 0.5, True)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(natural_hand, 0.5, True)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(RF_OK, 0.5, True)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(natural_hand, 0.5, True)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(LF_OK, 0.5, True)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(open_hand, 0.75, True)
rospy.sleep(0.75)

hand_commander.move_to_joint_value_target_unsafe(FF_Flex, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(MF_Flex, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(RF_Flex, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(LF_Flex, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(FF_Ext, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(MF_Ext, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(RF_Ext, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(LF_Ext, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(FF_Flex, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(MF_Flex, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(RF_Flex, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(LF_Flex, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(FF_Ext, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(MF_Ext, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(RF_Ext, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(LF_Ext, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(FF_Flex, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(MF_Flex, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(RF_Flex, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(LF_Flex, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(FF_Ext, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(MF_Ext, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(RF_Ext, 0.4, False)
rospy.sleep(0.2)
hand_commander.move_to_joint_value_target_unsafe(LF_Ext, 0.4, False)
rospy.sleep(0.2)

hand_commander.move_to_joint_value_target_unsafe(natural_hand, 0.5, True)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(FF_OK, 0.5, True)
rospy.sleep(2)
hand_commander.move_to_joint_value_target_unsafe(open_hand, 4.0, True)
rospy.sleep(4)


from sr_robot_msgs.msg import sendupdate, joint, Biotac, BiotacAll, ShadowPST
from sensor_msgs.msg import *
from std_msgs.msg import Float64

#the threshold for pdc above which the tactile is considered "pressed"
PDC_THRESHOLD = 3000
#the threshold for the PSTs above which the tactile is considered "pressed"
PST_THRESHOLD = 420

class FancyDemo(object):
    # starting position for the hand (DON't use until reviewed. Should be executed in two movement sequences)
    start_pos_hand = [ joint(joint_name = "THJ1", joint_target = 0),
                       joint(joint_name = "THJ2", joint_target = 0),
                       joint(joint_name = "THJ3", joint_target = 0),
                       joint(joint_name = "THJ4", joint_target = 0),
                       joint(joint_name = "THJ5", joint_target = 0),
                       joint(joint_name = "FFJ0", joint_target = 0),
                       joint(joint_name = "FFJ3", joint_target = 0),
                       joint(joint_name = "FFJ4", joint_target = 0),
                       joint(joint_name = "MFJ0", joint_target = 0),
                       joint(joint_name = "MFJ3", joint_target = 0),
                       joint(joint_name = "MFJ4", joint_target = 0),
                       joint(joint_name = "RFJ0", joint_target = 0),
                       joint(joint_name = "RFJ3", joint_target = 0),
                       joint(joint_name = "RFJ4", joint_target = 0),
                       joint(joint_name = "LFJ0", joint_target = 0),
                       joint(joint_name = "LFJ3", joint_target = 0),
                       joint(joint_name = "LFJ4", joint_target = 0),
                       joint(joint_name = "LFJ5", joint_target = 0),
                       joint(joint_name = "WRJ1", joint_target = 0),
                       joint(joint_name = "WRJ2", joint_target = 0) ]
    # flex first finger
    flex_ff = [ joint(joint_name = "FFJ0", joint_target = 180),
		        joint(joint_name = "FFJ3", joint_target = 90),
        		joint(joint_name = "FFJ4", joint_target = 0) ]
    # extend first finger
    ext_ff = [ joint(joint_name = "FFJ0", joint_target = 0),
	           joint(joint_name = "FFJ3", joint_target = 0),
	           joint(joint_name = "FFJ4", joint_target = 0) ]
    # flex middle finger
    flex_mf = [ joint(joint_name = "MFJ0", joint_target = 180),
		        joint(joint_name = "MFJ3", joint_target = 90),
		        joint(joint_name = "MFJ4", joint_target = 0) ]
    # extend middle finger
    ext_mf = [ joint(joint_name = "MFJ0", joint_target = 0),
	           joint(joint_name = "MFJ3", joint_target = 0),
	           joint(joint_name = "MFJ4", joint_target = 0) ]
    # flex ring finger
    flex_rf = [ joint(joint_name = "RFJ0", joint_target = 180),
		        joint(joint_name = "RFJ3", joint_target = 90),
		        joint(joint_name = "RFJ4", joint_target = 0) ]
    # extend ring finger
    ext_rf = [ joint(joint_name = "RFJ0", joint_target = 0),
	           joint(joint_name = "RFJ3", joint_target = 0),
	           joint(joint_name = "RFJ4", joint_target = 0) ]
    # flex little finger
    flex_lf = [ joint(joint_name = "LFJ0", joint_target = 180),
		        joint(joint_name = "LFJ3", joint_target = 90),
		        joint(joint_name = "LFJ4", joint_target = 0) ]
    # extend little finger
    ext_lf = [ joint(joint_name = "LFJ0", joint_target = 0),
	           joint(joint_name = "LFJ3", joint_target = 0),
	           joint(joint_name = "LFJ4", joint_target = 0) ]
    # flex thumb step 1
    flex_th_1 = [ joint(joint_name = "THJ1", joint_target = 0),
		          joint(joint_name = "THJ2", joint_target = 0),
		          joint(joint_name = "THJ3", joint_target = 0),
		          joint(joint_name = "THJ4", joint_target = 70),
		          joint(joint_name = "THJ5", joint_target = 0) ]
    # flex thumb step 2
    flex_th_2 = [ joint(joint_name = "THJ1", joint_target = 20),
		          joint(joint_name = "THJ2", joint_target = 40),
		          joint(joint_name = "THJ3", joint_target = 10),
		          joint(joint_name = "THJ4", joint_target = 70),
		          joint(joint_name = "THJ5", joint_target = 55) ]
    # extend thumb step 1
    ext_th_1 = [ joint(joint_name = "THJ1", joint_target = 20),
	             joint(joint_name = "THJ2", joint_target = 8),
	             joint(joint_name = "THJ3", joint_target = 0),
	             joint(joint_name = "THJ4", joint_target = 70),
	             joint(joint_name = "THJ5", joint_target = 0) ]
    # extend thumb step 2
    ext_th_2 = [ joint(joint_name = "THJ1", joint_target = 0),
	             joint(joint_name = "THJ2", joint_target = 0),
	             joint(joint_name = "THJ3", joint_target = 0),
	             joint(joint_name = "THJ4", joint_target = 0),
	             joint(joint_name = "THJ5", joint_target = -50) ]
    # zero thumb
    zero_th = [ joint(joint_name = "THJ1", joint_target = 0),
		        joint(joint_name = "THJ2", joint_target = 0),
		        joint(joint_name = "THJ3", joint_target = 0),
		        joint(joint_name = "THJ4", joint_target = 0),
		        joint(joint_name = "THJ5", joint_target = 0) ]

    # Pre O.K. with first finger
    pre_ff_ok = [ joint(joint_name = "THJ4", joint_target = 50) ]
    # O.K. with first finger
    ff_ok = [ joint(joint_name = "FFJ0", joint_target = 94),
	          joint(joint_name = "FFJ3", joint_target = 37),
	          joint(joint_name = "FFJ4", joint_target = -0.2),
	          joint(joint_name = "MFJ0", joint_target = 42),
    	          joint(joint_name = "MFJ3", joint_target = 33),
	          joint(joint_name = "MFJ4", joint_target = -3),
	          joint(joint_name = "RFJ0", joint_target = 50),
	          joint(joint_name = "RFJ3", joint_target = 18),
	          joint(joint_name = "RFJ4", joint_target = 0.5),
	          joint(joint_name = "LFJ0", joint_target = 30),
	          joint(joint_name = "LFJ3", joint_target = 0),
	          joint(joint_name = "LFJ4", joint_target = -6),
	          joint(joint_name = "LFJ5", joint_target = 7),	      
	          joint(joint_name = "THJ1", joint_target = 20),
	          joint(joint_name = "THJ2", joint_target = 20),
	          joint(joint_name = "THJ3", joint_target = 0),
	          joint(joint_name = "THJ4", joint_target = 50),
	          joint(joint_name = "THJ5", joint_target = 11) ]
    # O.K. transition from first finger to middle finger
    ff2mf_ok = [ joint(joint_name = "FFJ0", joint_target = 13.6),
	             joint(joint_name = "FFJ3", joint_target = 7),
	             joint(joint_name = "FFJ4", joint_target = -0.4),
	             joint(joint_name = "MFJ0", joint_target = 42),
	             joint(joint_name = "MFJ3", joint_target = 33),
	             joint(joint_name = "MFJ4", joint_target = -3),
	             joint(joint_name = "RFJ0", joint_target = 50),
	             joint(joint_name = "RFJ3", joint_target = 18),
	             joint(joint_name = "RFJ4", joint_target = 0.5),
	             joint(joint_name = "LFJ0", joint_target = 30),
	             joint(joint_name = "LFJ3", joint_target = 0),
	             joint(joint_name = "LFJ4", joint_target = -6),
	             joint(joint_name = "LFJ5", joint_target = 7),	      
	             joint(joint_name = "THJ1", joint_target = 40),
	             joint(joint_name = "THJ2", joint_target = 12),
	             joint(joint_name = "THJ3", joint_target = -10),
	             joint(joint_name = "THJ4", joint_target = 50),
	             joint(joint_name = "THJ5", joint_target = 2) ]
    # O.K. with middle finger
    mf_ok = [ joint(joint_name = "FFJ0", joint_target = 13.6),
	          joint(joint_name = "FFJ3", joint_target = 7),
	          joint(joint_name = "FFJ4", joint_target = -0.4),
	          joint(joint_name = "MFJ0", joint_target = 89),
	          joint(joint_name = "MFJ3", joint_target = 51),
	          joint(joint_name = "MFJ4", joint_target = 8),
	          joint(joint_name = "RFJ0", joint_target = 50),
	          joint(joint_name = "RFJ3", joint_target = 19),
	          joint(joint_name = "RFJ4", joint_target = -10),
	          joint(joint_name = "LFJ0", joint_target = 30),
	          joint(joint_name = "LFJ3", joint_target = 0),
	          joint(joint_name = "LFJ4", joint_target = -6),
	          joint(joint_name = "LFJ5", joint_target = 7),	      
	          joint(joint_name = "THJ1", joint_target = 20),
	          joint(joint_name = "THJ2", joint_target = 14),
	          joint(joint_name = "THJ3", joint_target = 7),
	          joint(joint_name = "THJ4", joint_target = 66),
	          joint(joint_name = "THJ5", joint_target = 23) ]
    # O.K. transition from middle finger to ring finger
    mf2rf_ok = [ joint(joint_name = "FFJ0", joint_target = 13.6),
                 joint(joint_name = "FFJ3", joint_target = 7),
	             joint(joint_name = "FFJ4", joint_target = -0.4),
	             joint(joint_name = "MFJ0", joint_target = 45),
	             joint(joint_name = "MFJ3", joint_target = 3.7),
       	         joint(joint_name = "MFJ4", joint_target = -1),
	             joint(joint_name = "RFJ0", joint_target = 50),
	             joint(joint_name = "RFJ3", joint_target = 18),
                 joint(joint_name = "RFJ4", joint_target = -14),
                 joint(joint_name = "LFJ0", joint_target = 30),
	             joint(joint_name = "LFJ3", joint_target = 0),
	             joint(joint_name = "LFJ4", joint_target = -6),
                 joint(joint_name = "LFJ5", joint_target = 7),	      
	             joint(joint_name = "THJ1", joint_target = 45),
	             joint(joint_name = "THJ2", joint_target = 8),
	             joint(joint_name = "THJ3", joint_target = 12),
	             joint(joint_name = "THJ4", joint_target = 64),
	             joint(joint_name = "THJ5", joint_target = 13.2) ]
    # O.K. with ring finger
    rf_ok = [ joint(joint_name = "FFJ0", joint_target = 13.6),
              joint(joint_name = "FFJ3", joint_target = 7),
	          joint(joint_name = "FFJ4", joint_target = -0.4),
	          joint(joint_name = "MFJ0", joint_target = 45),
	          joint(joint_name = "MFJ3", joint_target = 3.7),
	          joint(joint_name = "MFJ4", joint_target = -1),
	          joint(joint_name = "RFJ0", joint_target = 108),
	          joint(joint_name = "RFJ3", joint_target = 34),
	          joint(joint_name = "RFJ4", joint_target = -19),
	          joint(joint_name = "LFJ0", joint_target = 30),
	          joint(joint_name = "LFJ3", joint_target = 0),
	          joint(joint_name = "LFJ4", joint_target = -12),
	          joint(joint_name = "LFJ5", joint_target = 7),	      
	          joint(joint_name = "THJ1", joint_target = 20),
	          joint(joint_name = "THJ2", joint_target = 14),
	          joint(joint_name = "THJ3", joint_target = 15),
	          joint(joint_name = "THJ4", joint_target = 70),
	          joint(joint_name = "THJ5", joint_target = 37) ]
    # O.K. transition from ring finger to little finger
    rf2lf_ok = [ joint(joint_name = "FFJ0", joint_target = 13.6),
                 joint(joint_name = "FFJ3", joint_target = 7),
	             joint(joint_name = "FFJ4", joint_target = -0.4),
	             joint(joint_name = "MFJ0", joint_target = 45),
	             joint(joint_name = "MFJ3", joint_target = 3.7),
	             joint(joint_name = "MFJ4", joint_target = -1),
	             joint(joint_name = "RFJ0", joint_target = 74),
	             joint(joint_name = "RFJ3", joint_target = 6.5),
	             joint(joint_name = "RFJ4", joint_target = 0.5),
	             joint(joint_name = "LFJ0", joint_target = 30),
 	             joint(joint_name = "LFJ3", joint_target = 0),
	             joint(joint_name = "LFJ4", joint_target = -6),
	             joint(joint_name = "LFJ5", joint_target = 7),	      
	             joint(joint_name = "THJ1", joint_target = 42),
	             joint(joint_name = "THJ2", joint_target = 4.5),
	             joint(joint_name = "THJ3", joint_target = 7.7),
	             joint(joint_name = "THJ4", joint_target = 73),
	             joint(joint_name = "THJ5", joint_target = 21) ]
    # O.K. with little finger
    lf_ok = [ joint(joint_name = "FFJ0", joint_target = 13.6),
              joint(joint_name = "FFJ3", joint_target = 7),
	          joint(joint_name = "FFJ4", joint_target = -0.4),
	          joint(joint_name = "MFJ0", joint_target = 15),
	          joint(joint_name = "MFJ3", joint_target = 3.7),
	          joint(joint_name = "MFJ4", joint_target = -1),
	          joint(joint_name = "RFJ0", joint_target = 74),
	          joint(joint_name = "RFJ3", joint_target = 6.5),
	          joint(joint_name = "RFJ4", joint_target = 0.5),
	          joint(joint_name = "LFJ0", joint_target = 100),
	          joint(joint_name = "LFJ3", joint_target = 9),
	          joint(joint_name = "LFJ4", joint_target = -7.6),
	          joint(joint_name = "LFJ5", joint_target = 41),	      
	          joint(joint_name = "THJ1", joint_target = 40),
	          joint(joint_name = "THJ2", joint_target = 10),
	          joint(joint_name = "THJ3", joint_target = 10),
	          joint(joint_name = "THJ4", joint_target = 68),
	          joint(joint_name = "THJ5", joint_target = 25) ]
    # zero wrist
    zero_wr = [ joint(joint_name = "WRJ1", joint_target = 0),
		        joint(joint_name = "WRJ2", joint_target = 0) ]
    # north wrist
    n_wr = [ joint(joint_name = "WRJ1", joint_target = 15),
	         joint(joint_name = "WRJ2", joint_target = 0) ]
    # south wrist
    s_wr = [ joint(joint_name = "WRJ1", joint_target = -20),
	         joint(joint_name = "WRJ2", joint_target = 0) ]
    # east wrist
    e_wr = [ joint(joint_name = "WRJ1", joint_target = 0),
	     joint(joint_name = "WRJ2", joint_target = 8) ]
    # west wrist
    w_wr = [ joint(joint_name = "WRJ1", joint_target = 0),
	     joint(joint_name = "WRJ2", joint_target = -14) ]
    # northeast wrist
    ne_wr = [ joint(joint_name = "WRJ1", joint_target = 15),
	          joint(joint_name = "WRJ2", joint_target = 8) ]
    # northwest wrist
    nw_wr = [ joint(joint_name = "WRJ1", joint_target = 15),
    	      joint(joint_name = "WRJ2", joint_target = -14) ]
    # southweast wrist
    sw_wr = [ joint(joint_name = "WRJ1", joint_target = -20),
	          joint(joint_name = "WRJ2", joint_target = -14) ]
    # southeast wrist
    se_wr = [ joint(joint_name = "WRJ1", joint_target = -20),
	          joint(joint_name = "WRJ2", joint_target = 8) ]
    # grasp for shaking hands step 1
    shake_grasp_1 = [ joint(joint_name = "THJ1", joint_target = 0),
	        	      joint(joint_name = "THJ2", joint_target = 6),
	        	      joint(joint_name = "THJ3", joint_target = 10),
        		      joint(joint_name = "THJ4", joint_target = 37),
	        	      joint(joint_name = "THJ5", joint_target = 9),
		              joint(joint_name = "FFJ0", joint_target = 21),
        		      joint(joint_name = "FFJ3", joint_target = 26),
		              joint(joint_name = "FFJ4", joint_target = 0),
		              joint(joint_name = "MFJ0", joint_target = 18),
		              joint(joint_name = "MFJ3", joint_target = 24),
		              joint(joint_name = "MFJ4", joint_target = 0),
		              joint(joint_name = "RFJ0", joint_target = 30),
		              joint(joint_name = "RFJ3", joint_target = 16),
		              joint(joint_name = "RFJ4", joint_target = 0),
		              joint(joint_name = "LFJ0", joint_target = 30),
		              joint(joint_name = "LFJ3", joint_target = 0),
		              joint(joint_name = "LFJ4", joint_target = -10),
		              joint(joint_name = "LFJ5", joint_target = 10) ]
    # grasp for shaking hands step 2
    shake_grasp_2 = [ joint(joint_name = "THJ1", joint_target = 21),
		              joint(joint_name = "THJ2", joint_target = 12),
                      joint(joint_name = "THJ3", joint_target = 10),
            	      joint(joint_name = "THJ4", joint_target = 42),
        		      joint(joint_name = "THJ5", joint_target = 21),
                      joint(joint_name = "FFJ0", joint_target = 75),
        		      joint(joint_name = "FFJ3", joint_target = 29),
            	      joint(joint_name = "FFJ4", joint_target = 0),
		              joint(joint_name = "MFJ0", joint_target = 85),
                      joint(joint_name = "MFJ3", joint_target = 29),
		              joint(joint_name = "MFJ4", joint_target = 0),
        		      joint(joint_name = "RFJ0", joint_target = 75),
        		      joint(joint_name = "RFJ3", joint_target = 41),
		              joint(joint_name = "RFJ4", joint_target = 0),
		              joint(joint_name = "LFJ0", joint_target = 100),
        		      joint(joint_name = "LFJ3", joint_target = 41),
        		      joint(joint_name = "LFJ4", joint_target = 0),
		              joint(joint_name = "LFJ5", joint_target = 0) ]
    # store step 1 PST
    store_1_PST = [ joint(joint_name = "THJ1", joint_target = 0),
	            joint(joint_name = "THJ2", joint_target = 0),
                joint(joint_name = "THJ3", joint_target = 0),
	            joint(joint_name = "THJ4", joint_target = 60),
	            joint(joint_name = "THJ5", joint_target = 0),
	            joint(joint_name = "FFJ0", joint_target = 180),
	            joint(joint_name = "FFJ3", joint_target = 90),
	            joint(joint_name = "FFJ4", joint_target = 0),
	            joint(joint_name = "MFJ0", joint_target = 180),
	            joint(joint_name = "MFJ3", joint_target = 90),
	            joint(joint_name = "MFJ4", joint_target = 0),
	            joint(joint_name = "RFJ0", joint_target = 180),
	            joint(joint_name = "RFJ3", joint_target = 90),
	            joint(joint_name = "RFJ4", joint_target = 0),
	            joint(joint_name = "LFJ0", joint_target = 180),
	            joint(joint_name = "LFJ3", joint_target = 90),
	            joint(joint_name = "LFJ4", joint_target = 0),
	            joint(joint_name = "LFJ5", joint_target = 0),
	            joint(joint_name = "WRJ1", joint_target = 0),
	            joint(joint_name = "WRJ2", joint_target = 0) ]
    # store step 2 PST
    store_2_PST = [ joint(joint_name = "THJ1", joint_target = 50),
	            joint(joint_name = "THJ2", joint_target = 12),
	            joint(joint_name = "THJ3", joint_target = 0),
	            joint(joint_name = "THJ4", joint_target = 60),
	            joint(joint_name = "THJ5", joint_target = 27),
	            joint(joint_name = "FFJ0", joint_target = 180),
	            joint(joint_name = "FFJ3", joint_target = 90),
	            joint(joint_name = "FFJ4", joint_target = 0),
	            joint(joint_name = "MFJ0", joint_target = 180),
	            joint(joint_name = "MFJ3", joint_target = 90),
	            joint(joint_name = "MFJ4", joint_target = 0),
	            joint(joint_name = "RFJ0", joint_target = 180),
	            joint(joint_name = "RFJ3", joint_target = 90),
	            joint(joint_name = "RFJ4", joint_target = 0),
	            joint(joint_name = "LFJ0", joint_target = 180),
	            joint(joint_name = "LFJ3", joint_target = 90),
	            joint(joint_name = "LFJ4", joint_target = 0),
	            joint(joint_name = "LFJ5", joint_target = 0),
	            joint(joint_name = "WRJ1", joint_target = 0),
	            joint(joint_name = "WRJ2", joint_target = 0) ]
    # store step 1 BioTac
    store_1_BioTac = [ joint(joint_name = "THJ1", joint_target = 0),
	            joint(joint_name = "THJ2", joint_target = 0),
                joint(joint_name = "THJ3", joint_target = 0),
	            joint(joint_name = "THJ4", joint_target = 30),
	            joint(joint_name = "THJ5", joint_target = 0),
	            joint(joint_name = "FFJ0", joint_target = 180),
	            joint(joint_name = "FFJ3", joint_target = 90),
	            joint(joint_name = "FFJ4", joint_target = 0),
	            joint(joint_name = "MFJ0", joint_target = 180),
	            joint(joint_name = "MFJ3", joint_target = 90),
	            joint(joint_name = "MFJ4", joint_target = 0),
	            joint(joint_name = "RFJ0", joint_target = 180),
	            joint(joint_name = "RFJ3", joint_target = 90),
	            joint(joint_name = "RFJ4", joint_target = 0),
	            joint(joint_name = "LFJ0", joint_target = 180),
	            joint(joint_name = "LFJ3", joint_target = 90),
	            joint(joint_name = "LFJ4", joint_target = 0),
	            joint(joint_name = "LFJ5", joint_target = 0),
	            joint(joint_name = "WRJ1", joint_target = 0),
	            joint(joint_name = "WRJ2", joint_target = 0) ]
    # store step 2 BioTac
    store_2_BioTac = [ joint(joint_name = "THJ1", joint_target = 20),
	            joint(joint_name = "THJ2", joint_target = 36),
	            joint(joint_name = "THJ3", joint_target = 0),
	            joint(joint_name = "THJ4", joint_target = 30),
	            joint(joint_name = "THJ5", joint_target = -3),
	            joint(joint_name = "FFJ0", joint_target = 180),
	            joint(joint_name = "FFJ3", joint_target = 90),
	            joint(joint_name = "FFJ4", joint_target = 0),
	            joint(joint_name = "MFJ0", joint_target = 180),
	            joint(joint_name = "MFJ3", joint_target = 90),
	            joint(joint_name = "MFJ4", joint_target = 0),
	            joint(joint_name = "RFJ0", joint_target = 180),
	            joint(joint_name = "RFJ3", joint_target = 90),
	            joint(joint_name = "RFJ4", joint_target = 0),
	            joint(joint_name = "LFJ0", joint_target = 180),
	            joint(joint_name = "LFJ3", joint_target = 90),
	            joint(joint_name = "LFJ4", joint_target = 0),
	            joint(joint_name = "LFJ5", joint_target = 0),
	            joint(joint_name = "WRJ1", joint_target = 0),
	            joint(joint_name = "WRJ2", joint_target = 0) ]
    # store step 3
    store_3 = [ joint(joint_name = "THJ1", joint_target = 0),
	           joint(joint_name = "THJ2", joint_target = 0),
		   joint(joint_name = "THJ3", joint_target = 0),
	           joint(joint_name = "THJ4", joint_target = 65),
	           joint(joint_name = "THJ5", joint_target = 0) ]
    # business card pre-zero position
    bc_pre_zero = [ joint(joint_name = "FFJ0", joint_target = 13.6),
	                joint(joint_name = "FFJ3", joint_target = 7),
	                joint(joint_name = "FFJ4", joint_target = -0.4),
	                joint(joint_name = "MFJ0", joint_target = 51.1),
	                joint(joint_name = "MFJ3", joint_target = 33),
	                joint(joint_name = "MFJ4", joint_target = 21),
	                joint(joint_name = "RFJ0", joint_target = 50),
	                joint(joint_name = "RFJ3", joint_target = 18),
	                joint(joint_name = "RFJ4", joint_target = -21),
	                joint(joint_name = "LFJ0", joint_target = 30),
	                joint(joint_name = "LFJ3", joint_target = 0),
	                joint(joint_name = "LFJ4", joint_target = -24),
	                joint(joint_name = "LFJ5", joint_target = 7),	      
	                joint(joint_name = "THJ1", joint_target = 15.2),
	                joint(joint_name = "THJ2", joint_target = 7.4),
	                joint(joint_name = "THJ3", joint_target = -4),
	                joint(joint_name = "THJ4", joint_target = 50),
	                joint(joint_name = "THJ5", joint_target = -16.8) ]
    # business card zero position
    bc_zero = [ joint(joint_name = "FFJ0", joint_target = 13.6),
	            joint(joint_name = "FFJ3", joint_target = 7),
	            joint(joint_name = "FFJ4", joint_target = -0.4),
	            joint(joint_name = "MFJ0", joint_target = 51.1),
	            joint(joint_name = "MFJ3", joint_target = 32),
	            joint(joint_name = "MFJ4", joint_target = 21),
	            joint(joint_name = "RFJ0", joint_target = 50),
	            joint(joint_name = "RFJ3", joint_target = 18),
	            joint(joint_name = "RFJ4", joint_target = -21),
	            joint(joint_name = "LFJ0", joint_target = 30),
	            joint(joint_name = "LFJ3", joint_target = 0),
	            joint(joint_name = "LFJ4", joint_target = -24),
	            joint(joint_name = "LFJ5", joint_target = 7),	      
	            joint(joint_name = "THJ1", joint_target = 17.2),
	            joint(joint_name = "THJ2", joint_target = 12),
	            joint(joint_name = "THJ3", joint_target = -4),
	            joint(joint_name = "THJ4", joint_target = 50),
	            joint(joint_name = "THJ5", joint_target = -13.6) ]
    # business card position 1
    bc_1 = [ joint(joint_name = "FFJ0", joint_target = 137),
	         joint(joint_name = "FFJ3", joint_target = 7) ]
    # business card position 2 #UNUSED
    bc_2 = [ joint(joint_name = "FFJ0", joint_target = 137),
	         joint(joint_name = "FFJ3", joint_target = 31) ]
    # business card position 3
    bc_3 = [ joint(joint_name = "FFJ0", joint_target = 137),
	         joint(joint_name = "FFJ3", joint_target = 58) ]
    # business card position 4
    bc_4 = [ joint(joint_name = "FFJ0", joint_target = 71),
	         joint(joint_name = "FFJ3", joint_target = 58) ]
    # business card position 5
    bc_5 = [ joint(joint_name = "FFJ0", joint_target = 180),
	         joint(joint_name = "FFJ3", joint_target = 58) ]
    # business card position 6
    bc_6 = [ joint(joint_name = "FFJ0", joint_target = 180),
	         joint(joint_name = "FFJ3", joint_target = 0) ]
    # business card position 7
    bc_7 = [ joint(joint_name = "FFJ0", joint_target = 0),
	         joint(joint_name = "FFJ3", joint_target = 0) ]
    # business card position 8
    bc_8 = [ joint(joint_name = "FFJ0", joint_target = 137),
	         joint(joint_name = "FFJ3", joint_target = 15) ]
    # business card position 9 #UNUSED
    bc_9 = [ joint(joint_name = "FFJ0", joint_target = 137),
	         joint(joint_name = "FFJ3", joint_target = 30) ]
    # business card position 10 #UNUSED
    bc_10 = [ joint(joint_name = "FFJ0", joint_target = 137),
	          joint(joint_name = "FFJ3", joint_target = 60) ]
    # business card position 11 #UNUSED
    bc_11 = [ joint(joint_name = "FFJ0", joint_target = 137),
	          joint(joint_name = "FFJ3", joint_target = 31) ]
    # business card position 12
    bc_12 = [ joint(joint_name = "FFJ0", joint_target = 137),
	          joint(joint_name = "FFJ3", joint_target = 58) ]
    # business card position 13
    bc_13 = [ joint(joint_name = "FFJ0", joint_target = 71),
	          joint(joint_name = "FFJ3", joint_target = 58) ]
    # business card position 14
    bc_14 = [ joint(joint_name = "MFJ3", joint_target = 64),
	          joint(joint_name = "FFJ4", joint_target = 20) ]
    # business card position 15
    bc_15 = [ joint(joint_name = "FFJ0", joint_target = 81 ),
	          joint(joint_name = "FFJ4", joint_target = 20),
	          joint(joint_name = "FFJ3", joint_target = 50),
              joint(joint_name = "THJ4", joint_target = 55),
	          joint(joint_name = "THJ5", joint_target = 20) ]
    # business card position 16
    bc_16 = [ joint(joint_name = "MFJ0", joint_target = 20),
	          joint(joint_name = "MFJ3", joint_target = 10),
	          joint(joint_name = "MFJ4", joint_target = 0) ]

    #A boolean used in this demo: set to true while an action is running
    # just so we don't do 2 actions at once
    action_running = mutex.mutex()

    def __init__(self):
        #A vector containing the different callbacks, in the same order
        # as the tactiles.
        self.fingers_pressed_functions = [self.ff_pressed, self.mf_pressed, self.rf_pressed,
                                          self.lf_pressed, self.th_pressed]

        #The hand publishers:
        # we use a dictionnary of publishers, because on the etherCAT hand
        # you have one publisher per controller.
        self.hand_publishers = self.create_hand_publishers()

        #send the start position to the hand
        #self.hand_publish(self.start_pos_hand)

        #wait for the node to be initialized and then go to the starting position
        time.sleep(1)
        rospy.loginfo("OK, ready for the demo")

        # We subscribe to the data being published by the biotac sensors.
        # self.sub_biotacs = rospy.Subscriber("/lh/tactile", BiotacAll, self.callback_biotacs, queue_size=1)
        # self.sub_psts   = rospy.Subscriber("/lh/tactile", ShadowPST, self.callback_psts, queue_size=1)

    def create_hand_publishers(self):
        """
        Creates a dictionnary of publishers to send the targets to the controllers
        on /sh_??j?_mixed_position_velocity_controller/command
        """
        hand_pub = {}

        for joint in ["FFJ0", "FFJ3", "FFJ4",
                      "MFJ0", "MFJ3", "MFJ4",
                      "RFJ0", "RFJ3", "RFJ4",
                      "LFJ0", "LFJ3", "LFJ4", "LFJ5",
                      "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
                      "WRJ1", "WRJ2" ]:
            hand_pub[joint] = rospy.Publisher('sh_'+joint.lower()+'_position_controller/command', Float64, latch=True)

        return hand_pub

    def hand_publish(self, pose):
        """
        Publishes the given pose to the correct controllers for the hand.
        The targets are converted in radians.
        """
        for joint in pose:
            self.hand_publishers[joint.joint_name].publish( math.radians(joint.joint_target) )

    def callback_biotacs(self, msg):
        """
        The callback function for the biotacs. Checks if one of the fingers
        was pressed (filter the noise). If it is the case, call the
        corresponding function.

        @msg is the message containing the biotac data
        """
        #loop through the five tactiles
        for index,tactile in enumerate(msg.tactiles):
            #here we're just checking pdc (the pressure)
            # to see if a finger has been pressed, but you have
            # access to the whole data from the sensor
            # (look at sr_robot_msgs/msg/Biotac.msg)
            if tactile.pdc >= PDC_THRESHOLD:
                # the tactile has been pressed, call the
                # corresponding function
                self.fingers_pressed_functions[index](tactile.pdc)

    def callback_psts(self, msg):
        """
        The callback function for the PSTs. Checks if one of the fingers
        was pressed (filter the noise). If it is the case, call the
        corresponding function.

        @msg is the message containing the biotac data
        """
        #loop through the five tactiles
        for index,tactile in enumerate(msg.pressure):
            #here we're just checking the pressure
            # to see if a finger has been pressed
            # 18456 is the value the PST takes when the sensor is not plugged in
            if tactile >= PST_THRESHOLD and tactile != 18896:
                # the tactile has been pressed, call the
                # corresponding function
                self.fingers_pressed_functions[index](tactile)

    def ff_pressed(self,data):
        """
        The first finger was pressed.

        @param data: the pressure value (pdc)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok the finger sensor was pressed
        #p = subprocess.Popen('beep')

        rospy.loginfo("FF touched, running basic demo ")

        #send the start position to the hand
        self.hand_publish( self.store_3 )
        time.sleep(1)
        self.hand_publish( self.start_pos_hand )
        time.sleep(1)
        self.hand_publish( self.flex_ff )
        time.sleep(1)
        self.hand_publish( self.ext_ff )
        time.sleep(1)
        self.hand_publish( self.flex_mf )
        time.sleep(1)
        self.hand_publish( self.ext_mf )
        time.sleep(1)
        self.hand_publish( self.flex_rf )
        time.sleep(1)
        self.hand_publish( self.ext_rf )
        time.sleep(1)
        self.hand_publish( self.flex_lf )
        time.sleep(1)
        self.hand_publish( self.ext_lf )
        time.sleep(1)
        self.hand_publish( self.flex_th_1 )
        time.sleep(1)
        self.hand_publish( self.flex_th_2 )
        time.sleep(1)
        self.hand_publish( self.ext_th_1 )
        time.sleep(1)
        self.hand_publish( self.ext_th_2 )
        time.sleep(1)
        self.hand_publish( self.n_wr )
        time.sleep(1)
        self.hand_publish( self.s_wr )
        time.sleep(1)
        self.hand_publish( self.zero_wr )
        time.sleep(1)
        self.hand_publish( self.e_wr )
        time.sleep(1)
        self.hand_publish( self.w_wr )
        time.sleep(1)
        self.hand_publish( self.zero_wr )
        time.sleep(1)
        self.hand_publish( self.pre_ff_ok )
        time.sleep(0.3)
        self.hand_publish( self.ff_ok )
        time.sleep(1)
        self.hand_publish( self.ff2mf_ok )
        time.sleep(1)
        self.hand_publish( self.mf_ok )
        time.sleep(1)
        self.hand_publish( self.mf2rf_ok )
        time.sleep(1)
        self.hand_publish( self.rf_ok )
        time.sleep(1)
        self.hand_publish( self.rf2lf_ok )
        time.sleep(1)
        self.hand_publish( self.lf_ok )
        time.sleep(1)
        self.hand_publish( self.start_pos_hand )
        time.sleep(1)
        self.hand_publish( self.flex_ff )
        time.sleep(0.2)
        self.hand_publish( self.flex_mf )
        time.sleep(0.2)
        self.hand_publish( self.flex_rf )
        time.sleep(0.2)
        self.hand_publish( self.flex_lf )
        time.sleep(0.2)
        self.hand_publish( self.ext_ff )
        time.sleep(0.2)
        self.hand_publish( self.ext_mf )
        time.sleep(0.2)
        self.hand_publish( self.ext_rf )
        time.sleep(0.2)
        self.hand_publish( self.ext_lf )
        time.sleep(0.2)
        self.hand_publish( self.flex_ff )
        time.sleep(0.2)
        self.hand_publish( self.flex_mf )
        time.sleep(0.2)
        self.hand_publish( self.flex_rf )
        time.sleep(0.2)
        self.hand_publish( self.flex_lf )
        time.sleep(0.2)
        self.hand_publish( self.ext_ff )
        time.sleep(0.2)
        self.hand_publish( self.ext_mf )
        time.sleep(0.2)
        self.hand_publish( self.ext_rf )
        time.sleep(0.2)
        self.hand_publish( self.ext_lf )
        time.sleep(0.2)
        self.hand_publish( self.flex_ff )
        time.sleep(0.2)
        self.hand_publish( self.flex_mf )
        time.sleep(0.2)
        self.hand_publish( self.flex_rf )
        time.sleep(0.2)
        self.hand_publish( self.flex_lf )
        time.sleep(0.2)
        self.hand_publish( self.ext_ff )
        time.sleep(0.2)
        self.hand_publish( self.ext_mf )
        time.sleep(0.2)
        self.hand_publish( self.ext_rf )
        time.sleep(0.2)
        self.hand_publish( self.ext_lf )
        time.sleep(1.0)
        self.hand_publish( self.pre_ff_ok )
        time.sleep(0.3)
        self.hand_publish( self.ff_ok )
        time.sleep(1)
        self.hand_publish( self.ne_wr )
        time.sleep(1.2)
        self.hand_publish( self.nw_wr )
        time.sleep(1.2)
        self.hand_publish( self.sw_wr )
        time.sleep(1.2)
        self.hand_publish( self.se_wr )
        time.sleep(1.2)
        self.hand_publish( self.ne_wr )
        time.sleep(0.4)
        self.hand_publish( self.nw_wr )
        time.sleep(0.4)
        self.hand_publish( self.sw_wr )
        time.sleep(0.4)
        self.hand_publish( self.se_wr )
        time.sleep(0.4)
        self.hand_publish( self.zero_wr )
        time.sleep(1)
        self.hand_publish( self.start_pos_hand )
        time.sleep(1)
        
        #wait before next possible action
        time.sleep(.2)
        self.action_running.unlock()

    def mf_pressed(self, data):
        """
        The middle finger was pressed.
     
        @param data: the pressure value (pdc)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok finger was pressed
        #p = subprocess.Popen('beep')

        rospy.loginfo("MF touched, running business card demo ")

        #wait 1s for the user to release the sensor
        time.sleep(.2)

        #send the start position to the hand
        self.hand_publish( self.start_pos_hand )
        time.sleep(1)
        self.hand_publish( self.bc_pre_zero )
        time.sleep(2)
        self.hand_publish( self.bc_zero )
        time.sleep(3)
        self.hand_publish( self.bc_1 )
        time.sleep(1)
        #	self.hand_publish( self.bc_2 )
        #	time.sleep(1)
        self.hand_publish( self.bc_3 )
        time.sleep(1)
        self.hand_publish( self.bc_4 )
        time.sleep(1)
        self.hand_publish( self.bc_5 )
        time.sleep(1)
        self.hand_publish( self.bc_6 )
        time.sleep(1)
        self.hand_publish( self.bc_7 )
        time.sleep(1)
        self.hand_publish( self.bc_8 )
        time.sleep(1)
        #	self.hand_publish( self.bc_9 )
        #	time.sleep(1)
        #	self.hand_publish( self.bc_10 )
        #	time.sleep(1)
        #	self.hand_publish( self.bc_11 )
        #	time.sleep(1)
        self.hand_publish( self.bc_12 )
        time.sleep(1)
        self.hand_publish( self.bc_13 )
        time.sleep(1)
        #	self.hand_publish( self.bc_14 )
        #	time.sleep(1)
        self.hand_publish( self.bc_15 )
        time.sleep(1)
        self.hand_publish( self.bc_16 )
        time.sleep(3)
        self.hand_publish( self.start_pos_hand )
        
        #wait before next possible action
        time.sleep(3)
        self.action_running.unlock()


    def rf_pressed(self, data):
        """
        The ring finger was pressed.

        @param data: the pressure value (pdc)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok finger was pressed
        #p = subprocess.Popen('beep')

        rospy.loginfo("RF touched, running shaking hands demo.")

        #wait 1s for the user to release the sensor
        time.sleep(.2)

        #send the start position to the hand
        self.hand_publish( self.shake_grasp_1 )
        time.sleep(3)
        self.hand_publish( self.shake_grasp_2 )
        time.sleep(1)
        self.hand_publish( self.e_wr )
        time.sleep(0.33)
        self.hand_publish( self.w_wr )
        time.sleep(0.33)
        self.hand_publish( self.zero_wr )
        time.sleep(0.66)
        self.hand_publish( self.shake_grasp_1 )
        time.sleep(3)
        self.hand_publish( self.start_pos_hand )
        time.sleep(2)
        
        #wait before next possible action
        time.sleep(.2)
        self.action_running.unlock()

    def lf_pressed(self, data):
        """
        The little finger was pressed.

        @param data: the pressure value (pdc)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok finger pressed
        #p = subprocess.Popen('beep')

        rospy.loginfo("LF touched, going to store position.")

        #wait 1s for the user to release the sensor
        time.sleep(.2)

        #send the start position to the hand
        self.hand_publish( self.start_pos_hand )
        time.sleep(1)
        self.hand_publish( self.flex_lf )
        time.sleep(0.2)
        self.hand_publish( self.flex_rf )
        time.sleep(0.2)
        self.hand_publish( self.flex_mf )
        time.sleep(0.2)
        self.hand_publish( self.flex_ff )
        time.sleep(0.2)
        self.hand_publish( self.ext_lf )
        time.sleep(0.2)
        self.hand_publish( self.ext_rf )
        time.sleep(0.2)
        self.hand_publish( self.ext_mf )
        time.sleep(0.2)
        self.hand_publish( self.ext_ff )
        time.sleep(0.2)
        self.hand_publish( self.flex_lf )
        time.sleep(0.2)
        self.hand_publish( self.flex_rf )
        time.sleep(0.2)
        self.hand_publish( self.flex_mf )
        time.sleep(0.2)
        self.hand_publish( self.flex_ff )
        time.sleep(1)
        #self.hand_publish( self.store_1_PST )
        self.hand_publish( self.store_1_BioTac )
        time.sleep(1)
        #self.hand_publish( self.store_2_PST )
        self.hand_publish( self.store_2_BioTac )
        time.sleep(1)
        
        #wait before next possible action
        time.sleep(.2)
        self.action_running.unlock()

    def th_pressed(self, data):
        """
        The thumb was pressed.

        @param data: the pressure value (pdc)
        """
        #if we're already performing an action, don't do anything
        if not self.action_running.testandset():
            return

        #ok the finger was pressed
        #p = subprocess.Popen('beep')

        rospy.loginfo("TH touched, going to start position.")

        #wait 1s for the user to release the sensor
        time.sleep(.2)

        #send the thumb_up_position to the hand
        self.hand_publish( self.start_pos_hand )

        #wait before next possible action
        time.sleep(.2)
        self.action_running.unlock()

def main():
    """
    The main function
    """
    # init the ros node
    rospy.init_node('fancy_touch_demo', anonymous=True)

    fancy_demo = FancyDemo()

    # fancy_demo.ff_pressed(0)

    # subscribe until interrupted
    rospy.spin()


if __name__ == '__main__':
    main()

