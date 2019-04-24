cmake_minimum_required(VERSION 3.2)
project(ros_robodk_post_processors)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rospy
  rostest
  std_msgs
)

catkin_python_setup()

add_service_files(
  FILES
    # Common services to all post-processors
    MoveC.srv
    MoveJ.srv
    MoveL.srv
    Pause.srv
    ProgFinish.srv
    ProgSave.srv
    ProgSendRobot.srv
    ProgStart.srv
    RunCode.srv
    RunMessage.srv
    SetDO.srv
    SetGO.srv
    SetFrame.srv
    SetSpeed.srv
    SetSpeedJoints.srv
    SetTool.srv
    SetZoneData.srv
    WaitDI.srv

    # Motoman specifics
    Motoman/Arcof.srv
    Motoman/Arcon.srv
    Motoman/DontUseMFrame.srv
    Motoman/Macro.srv
    Motoman/SetFolder.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    rospy
    rostest
    std_msgs
)

#############
## Install ##
#############

# RoboDK post processors git submodule
install(
  DIRECTORY
    src/ros_robodk_post_processors/robodk_post_processors/
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}/ros_robodk_post_processors/robodk_post_processors/
)

install(
  DIRECTORY
    src/ros_robodk_post_processors/
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Services servers
install(
  PROGRAMS
    scripts/services
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  add_rostest(
    test/services.launch
    DEPENDENCIES
      ${PROJECT_NAME}_generate_messages_cpp
  )
endif()
