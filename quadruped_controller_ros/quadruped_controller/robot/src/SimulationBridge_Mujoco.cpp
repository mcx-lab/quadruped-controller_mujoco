#include "SimulationBridge_Mujoco.h"
#include <ros/console.h>
#include "Configuration.h"

void printUsage()
{
    printf(
        "Usage: ./mit_controller [robot-id]\n"
        "\t Where robot-id: a for Unitree A1 and A for Unitree Aliengo\n"
    );
}

bool SimulationBridge::sit_flag = false;
// Sets flag incase of SIGINT interruption - Ctrl+C
void SimulationBridge::sigHandler(int signum)
{
    printf("\nsigHandler called with %d \n", signum);
    
    /* wait till the robot sits down */
    sit_flag = true;
    usleep(5000000);

    flag = signum;
    
    /* Kill Ros */
    ros::shutdown();
}

SimulationBridge::SimulationBridge(int argc, char** argv, RobotController* robot_ctrl)
    : spinner(2),
      rate(500)
{  
    if(argc != 2) {
        printf("argc value: %d\n", argc);
        printUsage();
        assert(false);
    }

    if(argv[1][0] == 'a') {
        robotType = RobotType::UNITREE_A1;
    } else if (argv[1][0] == 'A') {
        robotType = RobotType::UNITREE_ALIENGO;
    } else {
        printUsage();
        assert(false);
    }
    
    _controller = robot_ctrl;
    _userControlParameters = robot_ctrl->getUserControlParameters();


    init();
}

void SimulationBridge::init()
{
    
    //Load Robot Parameters
    try {
      _robotParams.initializeFromYamlFile(THIS_COM "config/robot_params.yaml");
    } catch(std::exception& e) {
      printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
      exit(1);
    }

    if(!_robotParams.isFullyInitialized()) {
      printf("Failed to initialize all robot parameters\n");
      exit(1);
    }

    printf("Robot parameters loaded sucessfully\n");

    // Load User Parameters
    try {
        _userControlParameters->initializeFromYamlFile(THIS_COM "config/mc-mit-ctrl-user-parameters.yaml");
      } catch(std::exception& e) {
        printf("Failed to initialize user parameters from yaml file: %s\n", e.what());
        exit(1);
      }

      if(!_userControlParameters->isFullyInitialized()) {
        printf("Failed to initialize all user parameters\n");
        exit(1);
      }
    
    printf("User parameters loaded sucessfully\n");
    // _mujocoCommunicationChannel = new MujocoCommunicationChannel(robotType, &nh, &_lowCmd, &_lowState, &twist);
    _mujocoCommunicationChannel = new MujocoCommunicationChannel(&nh, &_lowCmd, &_lowState);
    ROS_INFO("Mujoco Communication Channel Created");
    
    _robotRunner = new RobotRunner(_controller, &nh, robotType, &taskManager, 0.002, "robot-control");
    // _robotRunner->robotType = robotType;
    _robotRunner->sit_flag = &sit_flag;
    _robotRunner->lowState = &_lowState;
    _robotRunner->lowCmd = &_lowCmd;
    _robotRunner->vectorNavData = &_vectorNavData;
    _robotRunner->controlParameters = &_robotParams;
    // _robotRunner->twist = &twist;
    ROS_INFO("Robot Runner Created");

    _robotRunner->init();

}

void SimulationBridge::run()
{
    signal (SIGINT, SimulationBridge::sigHandler);

    ROS_INFO("Starting Asynchronous Spinner");
    spinner.start();
    usleep(1000000);

    imuThread = std::thread(&SimulationBridge::readIMU, this);

    _robotRunner->start();

    // gazeboThread = std::thread(&SimulationBridge::pubGazebo, this);
    mujocoThread = std::thread(&SimulationBridge::pubMujoco, this);

    while(!flag)
    {
        usleep(2000);
    }

    // while(!flag)
    // {
    //     readIMU();

    //     if(_firstControllerRun)
    //     {
    //         _robotRunner->init();
    //         _firstControllerRun = false;
    //     }
    //     _robotRunner->run();

    //     pubGazebo();

    //     usleep(2000);
    // }

    // DEBUG
    std::cout << "Terminating parent thread\n";
    taskManager.stopAll();
    
    std::cout << "Terminating imu thread\n";
    imuThread.join();

    std::cout << "Terminating mujoco thread\n";
    mujocoThread.join();

}

void SimulationBridge::readIMU()
{
    while(ros::ok())
    {
        // _vectorNavData.accelerometer[0] = _lowState.imu.linear_acceleration.x;
        // _vectorNavData.accelerometer[1] = _lowState.imu.linear_acceleration.y;
        // _vectorNavData.accelerometer[2] = _lowState.imu.linear_acceleration.z;

        // _vectorNavData.quat[0] = _lowState.imu.orientation.x;
        // _vectorNavData.quat[1] = _lowState.imu.orientation.y;
        // _vectorNavData.quat[2] = lowStateMsg_.imu.orientation.z;
        // _vectorNavData.quat[3] = lowStateMsg_.imu.orientation.w;

        // _vectorNavData.gyro[0] = lowStateMsg_.imu.angular_velocity.x;
        // _vectorNavData.gyro[1] = lowStateMsg_.imu.angular_velocity.y;
        // _vectorNavData.gyro[2] = lowStateMsg_.imu.angular_velocity.z;

        _vectorNavData.accelerometer[0] = _lowState.imu.accelerometer[0];
        _vectorNavData.accelerometer[1] = _lowState.imu.accelerometer[1];
        _vectorNavData.accelerometer[2] = _lowState.imu.accelerometer[2];

        _vectorNavData.quat[0] = _lowState.imu.quaternion[1];
        _vectorNavData.quat[1] = _lowState.imu.quaternion[2];
        _vectorNavData.quat[2] = _lowState.imu.quaternion[3];
        _vectorNavData.quat[3] = _lowState.imu.quaternion[0];

        _vectorNavData.gyro[0] = _lowState.imu.gyroscope[0];
        _vectorNavData.gyro[1] = _lowState.imu.gyroscope[1];
        _vectorNavData.gyro[2] = _lowState.imu.gyroscope[2];
    }
    rate.sleep();

}

void SimulationBridge::pubMujoco()
{
    while(ros::ok())
    {
        _mujocoCommunicationChannel->sendServoCmd();

        //DEBUG
        if(_firstRun) 
        {
            printState();
            printCmd();

            _firstRun = false;
        }

        rate.sleep();
        
    }

}

SimulationBridge::~SimulationBridge()
{
    delete _mujocoCommunicationChannel;
    delete _robotRunner;
}

//DEBUG
void SimulationBridge::printState()
{
    ROS_INFO("Printing Robot State");
    for(int i=0; i<12; i++)
        std::cout << "\tmotorState[" << i <<"].q: " <<_lowState.motorState[i].q << std::endl;
    std::cout << std::endl;
}

void SimulationBridge::printCmd()
{
    ROS_INFO("Printing Controller Command");
    for(int i=0; i<12; i++)
        std::cout << "\tmotorCmd[" << i << "].q: " << _lowCmd.motorCmd[i].q << std::endl;
    std::cout << std::endl;
}
