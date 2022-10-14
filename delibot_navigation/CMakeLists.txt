cmake_minimum_required(VERSION 3.0.2)
project(delibot_navigation)

find_package(catkin REQUIRED)


################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)