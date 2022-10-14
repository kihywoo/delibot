cmake_minimum_required(VERSION 3.0.2)
project(delibot_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   PIgain.msg
   delibot_odom.msg
   encoder.msg
   imu.msg
   drive_info.msg
 )



## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES delibot_msgs
  CATKIN_DEPENDS geometry_msgs std_msgs
#  DEPENDS system_lib
)
