add_library(fsm_states SHARED 
	ControlFSM.cpp  
	ControlFSMData.h  
	ControlFSM.h  
	FSM_State.cpp  
	FSM_State.h  
	FSM_State_Passive.cpp  
	FSM_State_Passive.h  
	FSM_State_StandUp.cpp  
	FSM_State_StandUp.h
	FSM_State_Locomotion.cpp
	FSM_State_Locomotion.h  
	FSM_State_JointPD.cpp
	FSM_State_JointPD.h
	FSM_State_BalanceStand.h
	FSM_State_BalanceStand.cpp
	SafetyChecker.cpp  
	SafetyChecker.h
)

target_link_libraries(fsm_states a1_servo)

target_include_directories(fsm_states PUBLIC 
	${PROJECT_SOURCE_DIR}/
	${PROJECT_SOURCE_DIR}/third-party/
	${PROJECT_SOURCE_DIR}/mit_ctrl/
	)

#TODO-MPC link cMPC lib with fsm_states
