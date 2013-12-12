#set to 1 to compile sr_hand and sr_tactiles with the gazebo interface
export GAZEBO=1

#set to 1 if you dont want gazebo to start with the GUI
export GAZEBO_NO_X=0

# set to 1 for one finger hands
export ONE_FINGER=0

# set to 1 for left hands
export LEFT_HAND=0

# set to 1 if you have a CAN hand
export REAL_HAND=0

# set to 1 if you have a CAN arm
export REAL_ARM=0

#set to 1 if you have a muscle arm or hand
export MUSCLE=0

#set to 1 if you want to build the compatibility interface
# for the etherCAT hand (makes it possible to run  your programs
# developed for the CAN Hand on the etherCAT hand)
# default is 1 as it just builds an other node.
export ETHERCAT=1

#port in which the etherCAT hand is plugged in
#export ETHERCAT_PORT=eth1

#set to 1 if you're using PWM control on the etherCAT hand motors by default
export PWM_CONTROL=1

#set to 1 if you want to have access to the internal firmware repository
#NOTE: for Shadow employees only for the time being
export INTERNAL_FIRMWARE=0

#set to 1 if you have ellipsoid fingertips (ATI nano sensors)
export ELLIPSOID=0

#set to 1 if you want to use the simulation for the manipulation stack
export SIMULATED=1

# set to 1 for biotac hands (hands with biotac sensors)
export BIOTAC_HAND=1
