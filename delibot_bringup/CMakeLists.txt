cmake_minimum_required(VERSION 3.0.2)
project(delibot_bringup)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
  delibot_msgs
)

catkin_package(
  LIBRARIES delibot_bringup
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf sensor_msgs std_msgs delibot_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_publisher src/tf_publisher.cpp)
add_dependencies(tf_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_publisher src/test_publisher.cpp)
add_dependencies(test_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(odom_publisher src/odom_publisher.cpp)
add_dependencies(odom_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(tf_publisher
   ${catkin_LIBRARIES}
 )

target_link_libraries(test_publisher
   ${catkin_LIBRARIES}
 )

 target_link_libraries(odom_publisher
   ${catkin_LIBRARIES}
 )

catkin_install_python(PROGRAMS scripts/delibot_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
