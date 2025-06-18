#add_library
#targer_include_directory(common/include)
#target_library(common)
#target_library(fsmstates)
#thirdparty

# include_directories(${PROJECT_SOURCE_DIR}/common/include/)

add_library(convexMPC SHARED
	convexMPC_interface.cpp  
	convexMPC_interface.h  
	ConvexMPCLocomotion.cpp  
	ConvexMPCLocomotion.h  
	Gait.cpp  
	Gait.h  
	RobotState.cpp  
	RobotState.h  
	SolverMPC.cpp  
	SolverMPC.h )


target_link_libraries(convexMPC PUBLIC 
	biomimetics 
	qpOASES 
	JCQP #CHECK
	)
target_include_directories(convexMPC PUBLIC
	${PROJECT_SOURCE_DIR}/third-party/
	${PROJECT_SOURCE_DIR}/third-party/qpOASES/include
	${PROJECT_SOURCE_DIR}/common/include/
	)