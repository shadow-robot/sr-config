#!/usr/bin/env python
# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from sr_robot_commander.sr_hand_commander import SrHandCommander
import rospy
from burn_in_states import warehouse_states
from sr_robot_commander.sr_robot_state_exporter import SrRobotStateExporter

rospy.init_node("burn_in_test")

commander = SrHandCommander()
exporter = SrRobotStateExporter(warehouse_states)

commander.move_to_named_target(warehouse_states["burn_4_hand"])

test_trajectory = exporter.convert_trajectory([
    {
        'name': "burn_1_hand",
        "interpolate_time": 0.5,
        'pause_time': 1
    },
    {
        'name': "burn_2_hand",
        "interpolate_time": 0.5,
        'pause_time': 1
    },
    {
        'name': "burn_3_hand",
        "interpolate_time": 0.5,
        'pause_time': 0.5
    },
    {
        'name': "burn_4_hand",
        "interpolate_time": 0.5,
        'pause_time': 0.5
    },
    {
        'name': "burn_5_hand",
        "interpolate_time": 0.5,
        'pause_time': .5
    },
    {
        'name': "burn_4_hand",
        "interpolate_time": 0.5,
        'pause_time': .5
    }
])

while not rospy.is_shutdown():
    commander.run_named_trajectory(test_trajectory)
