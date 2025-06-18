#ifndef ROBOT_RUNNER_H
#define ROBOT_RUNNER_H

#include "ControlParameters/ControlParameters.h"
#include "ControlParameters/RobotParameters.h"
#include "Utilities/PeriodicTask.h"
#include "Controllers/LegController.h"
#include "Types/IMUTypes.h"
#include <geometry_msgs/Twist.h>
#include "Dynamics/UnitreeA1.h"
#include "Dynamics/Aliengo.h"
#include "JPosInitializer.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/PositionVelocityEstimator.h"
#include "RobotController.h"

#include <ros/ros.h>
#include "unitree_legged_msgs/StateEstimate.h"


class RobotRunner : public PeriodicTask {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //For Periodic running of RobotRunner
    RobotRunner(RobotController* , ros::NodeHandle*, RobotType, PeriodicTaskManager*, float, std::string);
    using PeriodicTask::PeriodicTask;
    void init() override;
    void run() override;
    void initializeStateEstimator();

    void run_jpos();

    //DEBUG
    void run_controller();
    void balance_standup();
    
    void cleanup() override;
    virtual ~RobotRunner();

    unitree_legged_msgs::LowState* lowState;
	unitree_legged_msgs::LowCmd* lowCmd;

    geometry_msgs::Twist* twist;
    
    VectorNavData* vectorNavData;
    RobotControlParameters* controlParameters;

    RobotController* _robot_ctrl;

    RobotType robotType;
    std::string robotName;

    /* For Sitting on SIGINT */
    bool* sit_flag;
    void run_sit_jpos();

private: 
    JPosInitializer<float>* _jpos_initializer;
    JPosInitializer<float>* _sit_jpos_initializer;
    LegController<float>* _legController = nullptr;
    StateEstimate<float> _stateEstimate;
    StateEstimatorContainer<float>* _stateEstimator;
    Quadruped<float> _quadruped;
    FloatingBaseModel<float> _model;
    DesiredStateCommand<float>* _desiredStateCommand;

    /* ROS */
    unitree_legged_msgs::StateEstimate _stateEstimateROS;
    ros::Publisher stateEstimatePub;

    void pubStateEstimate();
    
    //TEST
    double duration;
    int loop_count;
    double initPos[12], percent;
    static double targetPos[12];
    static double sitPos[12];

    Mat3<float> kpMat;
    Mat3<float> kdMat;   
    
    void setupStep();
    void finalizeStep();

    void updateDesiredStateCommand();

    //Making RobotRunner un-copyable
    RobotRunner(const RobotRunner& );
    RobotRunner& operator = (const RobotRunner& );

};

#endif 