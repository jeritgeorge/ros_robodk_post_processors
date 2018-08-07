cmake_minimum_required(VERSION 2.8.3)
project(ros_robodk_post_processors)
add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  rospy
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  GenerateNativeCode.srv
  MoveJ.srv
  MoveL.srv
  ProgFinish.srv
  ProgStart.srv
  RunMessage.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  rospy
  std_msgs
)

#############
## Install ##
#############

install(
  PROGRAMS
  scripts/generate_native_code_program.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
