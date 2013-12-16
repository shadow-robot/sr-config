# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_cyberglove_config)

find_package(catkin REQUIRED )


# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS # TODO add dependencies
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY calibrations/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/calibrations)

install(DIRECTORY mappings/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mappings)

install(DIRECTORY serial/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/serial)


