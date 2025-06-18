cmake_minimum_required(VERSION 3.0.2)
project(ocs2_quadruped_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  geometry_msgs
  std_msgs
)

add_message_files(
  FILES
  JointCommand.msg
  JointCommandArray.msg
  JointState.msg  
  LowState.msg
  RobotState.msg
  WbcTrackingErrors.msg
  FootForces.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs 
  geometry_msgs

)

#############
## Install ##
#############

# Mark topic names header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)