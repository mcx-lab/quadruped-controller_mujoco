# cmake_minimum_required(VERSION 3.16)
# project(unitree_mujoco)

# enable_language(C)
# enable_language(CXX)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
# find_package(mujoco REQUIRED)
# find_package(unitree_sdk2 REQUIRED)


# FILE (GLOB SIM_SRC
#         src/joystick/joystick.cc
#         src/mujoco/*.cc
#         src/unitree_sdk2_bridge/*.cc)

# set(SIM_DEPENDENCIES 
#     pthread
#     mujoco::mujoco
#     glfw
#     yaml-cpp
#     unitree_sdk2)


# add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
# target_link_libraries(unitree_mujoco  ${SIM_DEPENDENCIES})

# add_executable(test test/test_unitree_sdk2.cpp)
# target_link_libraries(test unitree_sdk2)

# add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)

# SET(CMAKE_BUILD_TYPE Release)





cmake_minimum_required(VERSION 3.0.2)
project(unitree_mujoco)

## 使用 C++17
set(CMAKE_CXX_STANDARD 17)

## 找 ROS 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  unitree_legged_msgs
)

# 如果你还依赖了非ROS库，比如 mujoco、unitree_sdk2
find_package(mujoco REQUIRED)
find_package(unitree_sdk2 REQUIRED)

## 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${mujoco_INCLUDE_DIRS}
  ${unitree_sdk2_INCLUDE_DIRS}
)

## 声明 catkin 包
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs unitree_legged_msgs
)

## 源文件
file(GLOB SIM_SRC
  src/joystick/joystick.cc
  src/mujoco/*.cc
  src/unitree_sdk2_bridge/*.cc
)

## 可执行文件
add_executable(unitree_mujoco src/main.cc ${SIM_SRC})
target_link_libraries(unitree_mujoco
  ${catkin_LIBRARIES}
  mujoco::mujoco
  unitree_sdk2
  pthread
  glfw
  yaml-cpp
)

install(TARGETS unitree_mujoco
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# add_executable(unitree_sdk2_test test/test_unitree_sdk2.cpp)
# target_link_libraries(unitree_sdk2_test test ${catkin_LIBRARIES} unitree_sdk2)

# add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
# target_link_libraries(jstest ${catkin_LIBRARIES})
