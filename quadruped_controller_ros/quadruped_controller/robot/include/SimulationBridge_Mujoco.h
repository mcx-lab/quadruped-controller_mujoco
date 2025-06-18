#ifndef _PROJECT_SIMULATION_BRIDGE_
#define _PROJECT_SIMULATION_BRIDGE_

#include "MujocoCommunicationChannel_2.h"
#include "RobotRunner.h"
#include <signal.h>

/* Flag for catching SIGINT */
volatile sig_atomic_t flag;

/* Acts as a bridge between the controller and gazebo
 Contains RobotRunner - controller end
 and GazeboSimulationBridge - Gazebo end */
class SimulationBridge
{
private:
    
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner;
    ros::Rate rate;

    /* Robot Controller */
    RobotController* _controller = nullptr;

    /* State of the robot and command to robot */
    unitree_legged_msgs::LowCmd _lowCmd;
    unitree_legged_msgs::LowState _lowState;
    // unitree_go::msg::dds_::LowCmd_ low_cmd{};     // default init
    // unitree_go::msg::dds_::LowState_ low_state{}; // default init
    VectorNavData _vectorNavData;

    /* Teleop keyboard input */
    // geometry_msgs::Twist twist;

    /* Main periodic task manager */
    PeriodicTaskManager taskManager;

    MujocoCommunicationChannel* _mujocoCommunicationChannel = nullptr;
    RobotRunner* _robotRunner = nullptr;

    /* Making SimulationBridge non-copyable */
    SimulationBridge (const SimulationBridge& ); 
    SimulationBridge& operator = (const SimulationBridge& ); 

    static void sigHandler(int );

    RobotControlParameters _robotParams;
    ControlParameters* _userControlParameters = nullptr;
    
    bool _firstControllerRun = true; //For Robot Runner
    bool _firstRun = true; //DEBUG - For Simulation Bridge
    static bool sit_flag;

    RobotType robotType;

public:
    
    void robotRunnerReplacement();
    void run();
    void init();

    /* Tasks - Multithreading */ 
    void readIMU();
    void pubMujoco();
    

    SimulationBridge(int argc, char** argv, RobotController* robot_ctrl);
    ~SimulationBridge();

    //DEBUG
    size_t count;
    void printState();
    void printCmd();

    std::thread imuThread;
    // std::thread gazeboThread;
    std::thread mujocoThread;

};

#endif
