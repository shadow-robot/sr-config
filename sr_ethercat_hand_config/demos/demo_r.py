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

FF_OK = {'rh_FFJ1': 0.890117918517108, 'rh_FFJ2': 0.925024503556994, 'rh_FFJ3': 0.715584993317675, 'rh_FFJ4': 0.0349065850398865, 'rh_LFJ1': 0.244346095279206, 'rh_LFJ2': 0.314159265358979, 'rh_LFJ3': 0.558505360638185, 'rh_LFJ4': -0.279252680319092, 'rh_LFJ5': 0.139626340159546, 'rh_MFJ1': 0.331612557878922, 'rh_MFJ2': 0.401425727958695, 'rh_MFJ3': 0.0523598775598298, 'rh_MFJ4': 0.0174532925199432, 'rh_RFJ1': 0.261799387799149, 'rh_RFJ2': 0.314159265358979, 'rh_RFJ3': 0.436332312998582, 'rh_RFJ4': -0.0174532925199432, 'rh_THJ1': 0.349065850398865, 'rh_THJ2': 0.541052068118242, 'rh_THJ3': -0.0117318081092081, 'rh_THJ4': 0.994837673636767, 'rh_THJ5': 0.157079632679489, 'rh_WRJ1': 0.0349065850398865, 'rh_WRJ2': -0.0523598775598298}

MF_OK = {'rh_FFJ1': 0.401425727958695, 'rh_FFJ2': 0.506145483078355, 'rh_FFJ3': 0, 'rh_FFJ4': 0.0349065850398865, 'rh_LFJ1': 0.244346095279206, 'rh_LFJ2': 0.314159265358979, 'rh_LFJ3': 0.558505360638185, 'rh_LFJ4': -0.279252680319092, 'rh_LFJ5': 0.139626340159546, 'rh_MFJ1': 0.890117918517108, 'rh_MFJ2': 0.994837673636767, 'rh_MFJ3': 0.820304748437334, 'rh_MFJ4': 0, 'rh_RFJ1': 0.279252680319092, 'rh_RFJ2': 0.349065850398865, 'rh_RFJ3': 0.383972435438752, 'rh_RFJ4': -0.0174532925199432, 'rh_THJ1': 0.418879020478639, 'rh_THJ2': 0.628318530717958, 'rh_THJ3': 0.0349065850398865, 'rh_THJ4': 1.13446401379631, 'rh_THJ5': 0.209439510239319, 'rh_WRJ1': 0.0349065850398865, 'rh_WRJ2': -0.0349065850398865}

RF_OK = {'rh_FFJ1': 0.349065850398865, 'rh_FFJ2': 0.523598775598298, 'rh_FFJ3': 0, 'rh_FFJ4': 0.0349065850398865, 'rh_LFJ1': 0.191986217719376, 'rh_LFJ2': 0.349065850398865, 'rh_LFJ3': 0.471238898038468, 'rh_LFJ4': -0.104719755119659, 'rh_LFJ5': 0.0523598775598298, 'rh_MFJ1': 0.418879020478639, 'rh_MFJ2': 0.523598775598298, 'rh_MFJ3': 0.0698131700797731, 'rh_MFJ4': 0, 'rh_RFJ1': 1.1693705988362, 'rh_RFJ2': 1.32645023151569, 'rh_RFJ3': 0.436332312998582, 'rh_RFJ4': -0.0349065850398865, 'rh_THJ1': 0.279252680319092, 'rh_THJ2': 0.680678408277788, 'rh_THJ3': 0.191986217719376, 'rh_THJ4': 1.20427718387608, 'rh_THJ5': 0.558505360638185, 'rh_WRJ1': 0, 'rh_WRJ2': -0.0349065850398865}

LF_OK = {'rh_FFJ1': 0.331612557878922, 'rh_FFJ2': 0.488692190558412, 'rh_FFJ3': 0, 'rh_FFJ4': 0, 'rh_LFJ1': 0.750491578357561, 'rh_LFJ2': 0.785398163397448, 'rh_LFJ3': 0.558505360638185, 'rh_LFJ4': -0.0349065850398865, 'rh_LFJ5': 0.680678408277788, 'rh_MFJ1': 0.471238898038468, 'rh_MFJ2': 0.663225115757845, 'rh_MFJ3': 0.139626340159546, 'rh_MFJ4': 0.0174532925199432, 'rh_RFJ1': 0.541052068118242, 'rh_RFJ2': 0.680678408277788, 'rh_RFJ3': 0.139626340159546, 'rh_RFJ4': -0.139626340159546, 'rh_THJ1': 0.244346095279206, 'rh_THJ2': 0.279252680319092, 'rh_THJ3': 0.209439510239319, 'rh_THJ4': 1.22173047639603, 'rh_THJ5': 0.261799387799149, 'rh_WRJ1': 0.0349065850398865, 'rh_WRJ2': -0.0523598775598298}


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

