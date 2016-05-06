#!/usr/bin/env python

# Script to move the right hand into store position.

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("right_hand_demo", anonymous=False)

hand_commander = SrHandCommander(name="right_hand")

open_hand = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
             'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
             'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
             'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
             'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0,
             'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

natural_hand = {'rh_FFJ1': -0.015231161684773453, 'rh_FFJ2': 0.4707354376773168, 'rh_FFJ3': -0.020599036757731948, 'rh_FFJ4': 0.03846792953124516, 'rh_THJ4': 1.129553058484541, 'rh_THJ5': 0.17285609095207696, 'rh_THJ1': 0.270633081305584, 'rh_THJ2': 0.7282881900694186, 'rh_THJ3': -0.015406461537391941, 'rh_LFJ2': 0.5850595799574672, 'rh_LFJ3': 0.5636517849387783, 'rh_LFJ1': 0.03461076652259942, 'rh_LFJ4': -0.28529553092230801, 'rh_LFJ5': 0.14943387379671536, 'rh_RFJ4': -0.030063596923893404, 'rh_RFJ1': 0.005949986086344283, 'rh_RFJ2': 0.6274874215503423, 'rh_RFJ3': 0.44395101691519906, 'rh_MFJ1': 0.025355373778972734, 'rh_MFJ3': 0.07746660184405069, 'rh_MFJ2': 0.9605842806822085, 'rh_MFJ4': 0.02818898576846175, 'rh_WRJ2': -0.05968479660514132, 'rh_WRJ1': -0.016812134199197103}

FF_Ext = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0}
FF_Flex = {'rh_FFJ1': 1.5707, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707}
MF_Ext = {'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0}
MF_Flex = {'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707}
RF_Ext = {'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0}
RF_Flex = {'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707}
LF_Ext = {'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0}
LF_Flex = {'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707}
TH_Ext = {'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0}
TH_Flex = {'rh_THJ1': 0.7854, 'rh_THJ2': 0.262, 'rh_THJ3': 0.209, 'rh_THJ4': 1.222, 'rh_THJ5': 1.00}
WR1_Ext = {'rh_WRJ1': -0.349}
WR1_Flex = {'rh_WRJ1': 0.349}
WR2_Ext = {'rh_WRJ2': -0.349}
WR2_Flex = {'rh_WRJ2': 0.140}
WR_Zero = {'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

AbdAdd_0 = {'rh_FFJ4': 0.0, 'rh_MFJ4': 0.0, 'rh_RFJ4': 0.0, 'rh_LFJ4': 0.0}
AbdAdd_1 = {'rh_FFJ4': 0.0, 'rh_MFJ4': 0.0, 'rh_RFJ4': 0.0, 'rh_LFJ4': -0.349}
AbdAdd_2 = {'rh_FFJ4': 0.0, 'rh_MFJ4': 0.0, 'rh_RFJ4': -0.349, 'rh_LFJ4': -0.349}
AbdAdd_3 = {'rh_FFJ4': 0.0, 'rh_MFJ4': 0.349, 'rh_RFJ4': -0.349, 'rh_LFJ4': -0.349}
AbdAdd_4 = {'rh_FFJ4': 0.349, 'rh_MFJ4': 0.349, 'rh_RFJ4': -0.349, 'rh_LFJ4': -0.349}
AbdAdd_5 = {'rh_FFJ4': -0.349, 'rh_MFJ4': -0.349, 'rh_RFJ4': 0.349, 'rh_LFJ4': 0.349}
AbdAdd_6 = {'rh_FFJ4': -0.349, 'rh_MFJ4': 0.349, 'rh_RFJ4': -0.349, 'rh_LFJ4': -0.349}
AbdAdd_7 = {'rh_FFJ4': -0.349, 'rh_MFJ4': -0.349, 'rh_RFJ4': -0.349, 'rh_LFJ4': -0.349}
AbdAdd_8 = {'rh_FFJ4': -0.349, 'rh_MFJ4': -0.349, 'rh_RFJ4': 0.349, 'rh_LFJ4': -0.349}
AbdAdd_9 = {'rh_FFJ4': -0.349, 'rh_MFJ4': -0.349, 'rh_RFJ4': 0.349, 'rh_LFJ4': 0.349}

FF_OK = {'rh_FFJ1': 0.022515630316621627, 'rh_FFJ2': 1.2556036427998678, 'rh_FFJ3': 0.8447767823310574, 'rh_FFJ4': 0.04590339173466104, 'rh_THJ4': 1.0949407683205896, 'rh_THJ5': 0.14200564847861585, 'rh_THJ1': 0.3683921782143842, 'rh_THJ2': 0.6373691082133975, 'rh_THJ3': -0.0244610170861788, 'rh_LFJ2': 0.5841731260484406, 'rh_LFJ3': 0.5632873801031884, 'rh_LFJ1': 0.03461076652259942, 'rh_LFJ4': -0.28125963828545088, 'rh_LFJ5': 0.1443737718056313, 'rh_RFJ4': -0.02468880994975094, 'rh_RFJ1': 0.0047599888690754155, 'rh_RFJ2': 0.6264485350908219, 'rh_RFJ3': 0.44382005595085083, 'rh_MFJ1': 0.021644831274732823, 'rh_MFJ3': 0.06212910667048069, 'rh_MFJ2': 0.9476510773927296, 'rh_MFJ4': 0.024157387999199982, 'rh_WRJ2': -0.05952234866755241, 'rh_WRJ1': -0.017882833898283462}

MF_OK = {'rh_FFJ1': 0.01192003957938792, 'rh_FFJ2': 0.8779137365294785, 'rh_FFJ3': -0.04879151632568085, 'rh_FFJ4': 0.03850631221477557, 'rh_THJ4': 1.1353427378511094, 'rh_THJ5': 0.25622286887027657, 'rh_THJ1': 0.41978526202199823, 'rh_THJ2': 0.6871574647024502, 'rh_THJ3': 0.052341509424183565, 'rh_LFJ2': 0.5832866721394142, 'rh_LFJ3': 0.5626116821752853, 'rh_LFJ1': 0.0339451748587033, 'rh_LFJ4': -0.28131817973754372, 'rh_LFJ5': 0.14504552420005684, 'rh_RFJ4': -0.023773075709565508, 'rh_RFJ1': 0.004164990260441037, 'rh_RFJ2': 0.6212541027932197, 'rh_RFJ3': 0.4446185880909443, 'rh_MFJ1': 0.15027697142171648, 'rh_MFJ3': 0.8577206183860272, 'rh_MFJ2': 1.2647385897689567, 'rh_MFJ4': 0.007472558773148949, 'rh_WRJ2': -0.05935494962092569, 'rh_WRJ1': -0.00794173869246716}

RF_OK = {'rh_FFJ1': 0.012582264000465027, 'rh_FFJ2': 0.892676859901611, 'rh_FFJ3': -0.01668023341905365, 'rh_FFJ4': 0.03684708613889836, 'rh_THJ4': 1.3244270470831758, 'rh_THJ5': 0.5461264574235202, 'rh_THJ1': 0.04931913989452231, 'rh_THJ2': 0.7723593776592549, 'rh_THJ3': 0.2332251218233079, 'rh_LFJ2': 0.5380775227790644, 'rh_LFJ3': 0.5378242421019102, 'rh_LFJ1': 0.029286033211430307, 'rh_LFJ4': -0.17266088110429886, 'rh_LFJ5': 0.11609954153828721, 'rh_RFJ4': -0.1410235306590761, 'rh_RFJ1': 0.04343489843031345, 'rh_RFJ2': 1.4093381677689403, 'rh_RFJ3': 0.7687871061350975, 'rh_MFJ1': 0.03710542504239911, 'rh_MFJ3': 0.11668324341742378, 'rh_MFJ2': 0.9358936198568396, 'rh_MFJ4': 0.03313362638105717, 'rh_WRJ2': -0.06225971637785656, 'rh_WRJ1': -0.009767850763861427}

LF_OK = {'rh_FFJ1': 0.00397334652646264, 'rh_FFJ2': 0.8975979010256552, 'rh_FFJ3': -0.0016476274144017463, 'rh_FFJ4': 0.03250985089166586, 'rh_THJ4': 1.2267774543912607, 'rh_THJ5': 0.20310694069887214, 'rh_THJ1': 0.2888424727858525, 'rh_THJ2': 0.5641506091726277, 'rh_THJ3': 0.23278402932191808, 'rh_LFJ2': 1.4468938248187885, 'rh_LFJ3': 0.2733478661324566, 'rh_LFJ1': 0.01406673264624339, 'rh_LFJ4': -0.22573803931632768, 'rh_LFJ5': 0.7547867069581923, 'rh_RFJ4': -0.1442490766383741, 'rh_RFJ1': 0.03331992208352813, 'rh_RFJ2': 1.2615629035756921, 'rh_RFJ3': 0.158866717571428, 'rh_MFJ1': 0.03215803503674591, 'rh_MFJ3': 0.14292484557904367, 'rh_MFJ2': 0.9464753316391405, 'rh_MFJ4': 0.023358296483200973, 'rh_WRJ2': -0.05471159545568462, 'rh_WRJ1': -0.011152559328240493}


hand_commander.move_to_joint_value_target_unsafe(open_hand, 2.0, False)
rospy.sleep(2)
hand_commander.move_to_joint_value_target_unsafe(FF_Flex, 0.75, False)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(FF_Ext, 0.75, False)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(MF_Flex, 0.75, False)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(MF_Ext, 0.75, False)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(RF_Flex, 0.75, False)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(RF_Ext, 0.75, False)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(LF_Flex, 0.75, False)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(LF_Ext, 0.75, False)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(TH_Flex, 1.0, False)
rospy.sleep(1)
hand_commander.move_to_joint_value_target_unsafe(TH_Ext, 1.0, False)
rospy.sleep(1)
hand_commander.move_to_joint_value_target_unsafe(WR1_Flex, 0.75, False)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(WR1_Ext, 0.75, False)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(WR2_Flex, 0.75, False)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(WR2_Ext, 0.75, False)
rospy.sleep(0.75)
hand_commander.move_to_joint_value_target_unsafe(WR_Zero, 0.75, False)
rospy.sleep(1)

hand_commander.move_to_joint_value_target_unsafe(AbdAdd_0, 0.25, False)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_1, 0.25, False)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_2, 0.25, False)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_3, 0.25, False)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_4, 0.25, False)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_5, 0.25, False)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_4, 0.25, False)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_6, 0.25, False)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_7, 0.25, False)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_8, 0.25, False)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_9, 0.25, False)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_0, 0.25, False)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_7, 0.25, False)
rospy.sleep(0.25)
hand_commander.move_to_joint_value_target_unsafe(AbdAdd_0, 0.25, False)
rospy.sleep(0.25)

hand_commander.move_to_joint_value_target_unsafe(natural_hand, 0.5, False)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(FF_OK, 0.5, False)
rospy.sleep(1.0)
hand_commander.move_to_joint_value_target_unsafe(natural_hand, 0.5, False)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(MF_OK, 0.5, False)
rospy.sleep(1.0)
hand_commander.move_to_joint_value_target_unsafe(natural_hand, 0.5, False)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(RF_OK, 0.5, False)
rospy.sleep(1.0)
hand_commander.move_to_joint_value_target_unsafe(natural_hand, 0.5, False)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(LF_OK, 0.5, False)
rospy.sleep(1.0)
hand_commander.move_to_joint_value_target_unsafe(open_hand, 0.75, False)
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

hand_commander.move_to_joint_value_target_unsafe(natural_hand, 0.5, False)
rospy.sleep(0.5)
hand_commander.move_to_joint_value_target_unsafe(FF_OK, 0.5, False)
rospy.sleep(2)
hand_commander.move_to_joint_value_target_unsafe(open_hand, 4.0, False)
rospy.sleep(4)

