#this is a boolean to specify that the config has been installed locally
# it is use in the launch files
export SHADOW_ROS_CONFIG_INSTALLED=0

#the SR_CONFIG_BZR_BRANCH variable is set if a branch has
# already been created for this user.
export SR_CONFIG_BZR_BRANCH=''

#this is the path to the default installed config files
export SHADOW_ROS_CONFIG_PATH=/etc/robot

#set to 1 to compile sr_hand and sr_tactiles with the gazebo interface
export GAZEBO=1

# set to 1 for one finger hands
export ONE_FINGER=0

# set to 1 for left hands
export LEFT_HAND=0

# set to 1 if you have a CAN hand
export REAL_HAND=0

# set to 1 if you have a CAN arm
export REAL_ARM=0

#set to 1 if you have a muscle arm or hand
export VALVES=0

#set to 1 if you want to build the compatibility interface for the etherCAT hand
export ETHERCAT=0

#set to 1 if you want to build the compatibility interface for the etherCAT hand
export ETHERCAT_PORT=eth1

#set to 1 if you want to have access to the internal firmware repository
#NOTE: for Shadow employees only for the time being
export INTERNAL_FIRMWARE=0
