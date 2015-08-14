Hand Configuration
==================

* [Calibrations] - Both Left and Right hand calibration files, including the calibration of each joint and the 2 pressure sensors per joint. These are set by Shadow Robot prior to the delivery of a hand and should only be changed by a person that has received training for the calibration procedure.
* [Controls] - Yaml files containing definitions for calibration, effort, joint, position and motor controllers
* [Demos] - Demo files for store position (demo_rs.py or demo_ls.py), general demo (demo_r.py or demo_l.py), and card trick demo (demo_rc.py).
* [Launch] - Launch files for Left/Right/Bimanual hands, specific to an install
* [Mappings] - The file which provides the mapping between hand joints and actuators is specified here. 
* [Rates] - Sensor and actuator update rate files.

## Joint to actuator mappings
The mapping files for each type of hand can be found [here](https://github.com/shadow-robot/sr-ros-interface-ethercat/tree/indigo-devel/sr_edc_launch/mappings/default_mappings). The mapping which corresponds to the system that you are launching, i.e. right hand/left hand/motor actuated/muscle actuated, should be specified in the corresponding lh (left hand) or rh (right hand) load_joint_mapping.xml, in the mappings directory. 

Changing the joint to actuator mapping should be done with caution, as an incorrect mapping could cause damage to the hand.

## Launching a hand
This directory includes launch files for left hand/right hand/bimanual systems.

The ethernet port in which the hand is connected is specified in the launch file, if the port is changed, then it should be edited here. The hand_serial parameter is hand specific, so should only be changed if launching a different hand. The serial of a hand can be found by running the launch file with no serial specified, argument and then check the program output. You should see something like:
```bash
Trying to read mapping for: /hand/mapping/1178
```
To lauch a right hand, run:

```bash
roslaunch sr_ethercat_hand_config sr_rhand.launch
```
To launch a left hand:
```bash
roslaunch sr_ethercat_hand_config sr_lhand.launch
```
And for a bimanual system:
```bash
roslaunch sr_ethercat_hand_config sr_system.launch
```

## Running a demo
The demo scripts can be run via rosrun, with a command of the following format:

For the right hand:
```bash
rosrun sr_ethercat_hand_config demo_r.py
```

And for the left hand:
```bash
rosrun sr_ethercat_hand_config demo_l.py
```