#!/usr/bin/env python
 
import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
 
# Change name "script_demo_template_node" with name of your demo
rospy.init_node("IROS_fan_demo", anonymous=True)
 
hand_commander = SrHandCommander()
arm_commander = SrArmCommander()

rospy.sleep(2.0)

arm_commander.move_to_named_target("hand_receieves_fan_arm", True)

#define the trajectory
trajectory_hand_recieves_fan = [
    {
        'name': 'hand_receieves_fan_hand',
        'interpolate_time': 2.0
    }
]
hand_commander.run_named_trajectory_unsafe(trajectory_hand_recieves_fan, True)

rospy.sleep(6.0)

trajectory_hand_readjustment = [
    {
        'name': 'hand_grasps_fan_from_receiving_hand',
        'interpolate_time': 1.0
    },
]

hand_commander.run_named_trajectory_unsafe(trajectory_hand_readjustment, True)

arm_commander.move_to_named_target("hand_flick_arm", True)

trajectory_hand_flick = [
    {
        'name':'back_flick_hand',
        'interpolate_time': 0.2
    },
]

hand_commander.run_named_trajectory_unsafe(trajectory_hand_flick, True)

rospy.sleep(1.0)

arm_commander.move_to_named_target("large_turn_arm", True)

trajectory_large_turn = [
    {
        'name':'large_turn_hand',
        'interpolate_time': 0.2
    },
]

hand_commander.run_named_trajectory_unsafe(trajectory_large_turn, True)

rospy.sleep(0.5)

arm_commander.move_to_named_target("back_flick_arm", True)

trajectory_back_flick = [
    {
        'name':'back_flick_hand',
        'interpolate_time': 1.0
    }
]

hand_commander.run_named_trajectory_unsafe(trajectory_back_flick, True)

rospy.sleep(0.5)

arm_commander.move_to_named_target("grasp_fan_arm", True)

trajectory_grasp_fan = [
    {
        'name':'grasp_fan_hand',
        'interpolate_time': 0.4
    }
]

hand_commander.run_named_trajectory_unsafe(trajectory_grasp_fan, True)

trajectory_fan_pull_lf = [
    {
        'name':'fan_pull_lf_hand',
        'interpolate_time': 0.2
    }
]

hand_commander.run_named_trajectory_unsafe(trajectory_fan_pull_lf, True)

rospy.sleep(1.0)

arm_commander.move_to_named_target("lift_fan_arm", True)

rospy.sleep(1.0)

trajectory_fanning_fan = [
    {
        'name':'fanning_position_2_arm',
        'interpolate_time': 0.1
    },
    {
        'name':'fanning_position_3_arm',
        'interpolate_time': 1
    },
    {
        'name':'fanning_position_2_arm',
        'interpolate_time': 1.3
    },
    {
        'name':'fanning_position_3_arm',
        'interpolate_time': 1
    },
    {
        'name':'fanning_position_2_arm',
        'interpolate_time': 1.3
    },
    {
        'name':'fanning_position_3_arm',
        'interpolate_time': 1
    },
    {
        'name':'fanning_position_2_arm',
        'interpolate_time': 1.3
    },
    {
        'name':'fanning_position_3_arm',
        'interpolate_time': 1
    },
    {
        'name':'fanning_position_2_arm',
        'interpolate_time': 1.3
    }, 
]

arm_commander.run_named_trajectory_unsafe(trajectory_fanning_fan, True)

rospy.sleep(10.0)

arm_commander.move_to_named_target("hand_receieves_fan_arm", True)
