cmake_minimum_required(VERSION 2.8.3)
project(ros_robodk_post_processors)
add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
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
  MoveC.srv
  MoveJ.srv
  MoveL.srv
  Pause.srv
  ProgFinish.srv
  ProgStart.srv
  RunMessage.srv
  SetFrame.srv
  SetTool.srv
)

# Motoman specifics
add_service_files(
  FILES
  Motoman/Arcof.srv
  Motoman/Arcon.srv
  Motoman/Macro.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
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
  geometry_msgs
  message_runtime
  rospy
  std_msgs
)

#############
## Install ##
#############

# RoboDK post processors git submodule
install(
  DIRECTORY
  scripts/robodk_postprocessors/
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}/robodk_postprocessors/
)

# Services servers
install(
  PROGRAMS
  scripts/services.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
