cmake_minimum_required(VERSION 3.5.1)
project(quadruped_controller)

# find specific catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  genmsg
  joint_state_controller
  robot_state_publisher
  roscpp
  gazebo_ros
  std_msgs
  tf
  geometry_msgs
  unitree_legged_msgs
  message_generation
)


# 包含相关头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ocs2_quadruped_msgs_INCLUDE_DIRS}  # 包含 ocs2_quadruped_msgs 生成的头文件路径
)



# include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
# include_directories(/usr/local/include/iceoryx/v2.0.2)
set(UNITREE_LEGGED_LIB unitree_sdk2 rt pthread)

SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
    ${CMAKE_BINARY_DIR}/Configuration.h)

# # find gazebo packages
# find_package(gazebo REQUIRED)

# include_directories(/usr/local/include/ddscxx)
catkin_package(
  INCLUDE_DIRS
  include
  ${CATKIN_DEVEL_PREFIX}/include  # 添加这一行
  CATKIN_DEPENDS
  unitree_legged_msgs roscpp
)




include_directories(
  ${CATKIN_DEVEL_PREFIX}/include
  robot/include
  ${catkin_INCLUDE_DIRS}
)




# include_directories(/usr/local/include/ddscxx)
# catkin_package(
#   INCLUDE_DIRS include
#   # LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS ocs2_quadruped_msgs roscpp unitree_legged_msgs
#   # DEPENDS std_msgs sensor_msgs
# )

# catkin_package(
#  INCLUDE_DIRS 
#   include
#   /usr/local/include/ddscxx
#   #/opt/unitree_robotics/unitree_sdk2/thirdparty/include/ddscxx
#   #/usr/local/include/iceoryx/v2.0.2
#  LIBRARIES 
#  ${PROJECT_NAME}
#  CATKIN_DEPENDS ocs2_quadruped_msgs roscpp
#  DEPENDS 
#   std_msgs
#   sensor_msgs
# )

add_library(${PROJECT_NAME}
robot/src/MujocoCommunicationChannel_2.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${UNITREE_LEGGED_LIB}
)



target_include_directories(${PROJECT_NAME} PRIVATE ${CATKIN_DEVEL_PREFIX}/include)




include_directories(
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb")
set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
    -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
    -Wsign-promo") # -Werror -Wshadow
set(CMAKE_C_FLAGS "-O3  -ggdb -std=gnu99 -I.")

add_subdirectory(third-party)
add_subdirectory(common)
add_subdirectory(robot)
add_subdirectory(mit_ctrl)
add_subdirectory(fsm_states)

add_executable(mit_controller main.cpp MIT_Controller.cpp)
target_link_libraries(mit_controller a1_servo biomimetics fsm_states convexMPC WBC_Ctrl ${catkin_LIBRARIES})
