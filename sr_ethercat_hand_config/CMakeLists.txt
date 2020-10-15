# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_ethercat_hand_config)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED )

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
file(GLOB SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" 
   "${CMAKE_CURRENT_SOURCE_DIR}/demos/*")
install(PROGRAMS ${SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY calibrations/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/calibrations)

install(DIRECTORY controls/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/controls)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY mappings/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mappings)

install(DIRECTORY rates/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rates)

install(FILES diagnostic_analyzer.yaml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



