
include_directories("../third-party/ParamHandler")
include_directories("../third-party/osqp/include")       # osqp includes

add_library(
    biomimetics SHARED
        src/Controllers/LegController.cpp
        src/Controllers/OrientationEstimator.cpp
        src/Controllers/PositionVelocityEstimator.cpp
        src/Controllers/GaitScheduler.cpp
        src/Controllers/DesiredStateCommand.cpp
        src/Controllers/FootSwingTrajectory.cpp
        src/Dynamics/Quadruped.cpp
        src/Dynamics/FloatingBaseModel.cpp 
        src/Utilities/PeriodicTask.cpp
        src/Utilities/Utilities_print.cpp
        src/Utilities/utilities.cpp
        src/ControlParameters/ControlParameters.cpp
        src/FootstepPlanner/GraphSearch.cpp
        src/SparseCMPC/OsqpTriples.cpp 
        src/SparseCMPC/SparseCMPC_Math.cpp
        src/SparseCMPC/SparseCMPC.cpp
                                            )

target_include_directories(biomimetics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include )
target_link_libraries(biomimetics dynacore_param_handler JCQP osqp)
