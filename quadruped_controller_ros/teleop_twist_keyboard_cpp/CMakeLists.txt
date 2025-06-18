cmake_minimum_required(VERSION 2.8.3)
project(teleop_twist_keyboard_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(teleop_twist_keyboard src/teleop_twist_keyboardv2.cpp)
target_link_libraries(teleop_twist_keyboard ${catkin_LIBRARIES})

add_executable(turtlebot_controller robot/src/SimulationBridge.cpp robot/src/GazeboCommunicationChannel.cpp)
target_link_libraries(turtlebot_controller ${catkin_LIBRARIES})
target_include_directories(turtlebot_controller PUBLIC robot/include/)

# set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS " -O3 -no-pie -march=native -ggdb")
set(CMAKE_CXX_STANDARD 14)