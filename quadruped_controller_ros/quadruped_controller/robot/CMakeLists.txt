include_directories("../third-party")
include_directories("../third-party/ParamHandler")
include_directories("${CMAKE_BINARY_DIR}")

# add_library(a1_servo SHARED 
#     src/SimulationBridge.cpp 
#     src/GazeboCommunicationChannel.cpp 
#     src/RobotRunner.cpp
#     src/JPosInitializer.cpp
# )
add_library(a1_servo SHARED 
    src/SimulationBridge_Mujoco.cpp 
    src/MujocoCommunicationChannel_2.cpp 
    src/RobotRunner.cpp
    src/JPosInitializer.cpp
)
target_include_directories(a1_servo PUBLIC "include") # "../"

# add_dependencies(a1_servo unitree_legged_msgs_gencpp)
add_dependencies(a1_servo unitree_legged_msgs_gencpp)
target_link_libraries(a1_servo biomimetics dynacore_param_handler)
