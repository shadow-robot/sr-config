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
export MUSCLE=1

#set to 1 if you want to build the compatibility interface for the etherCAT hand
export ETHERCAT=0

#set to 1 if you want to build the compatibility interface for the etherCAT hand
export ETHERCAT_PORT=eth1

#set to 1 if you're using PWM control on the etherCAT hand motors by default
export PWM_CONTROL=1

#set to 1 if you want to have access to the internal firmware repository
#NOTE: for Shadow employees only for the time being
export INTERNAL_FIRMWARE=0

#set to 1 if you have ellipsoid fingertips (ATI nano sensors)
ELLIPSOID=0