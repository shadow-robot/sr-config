#!/usr/bin/env python3

# Copyright 2019, 2022 Shadow Robot Company Ltd.
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
from burn_in_states import warehouse_states
from sr_robot_commander.sr_robot_state_exporter import SrRobotStateExporter
from sr_robot_commander.sr_hand_commander import SrHandCommander

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
