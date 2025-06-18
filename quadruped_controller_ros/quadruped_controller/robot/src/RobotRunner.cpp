#include "RobotRunner.h"

/* Standing */
double RobotRunner::targetPos[12] = {
                         0.0, -0.67, 1.3,
                        -0.0, -0.67, 1.3, 
                         0.0, -0.67, 1.3, 
                        -0.0, -0.67, 1.3
                                         };

/* Sitting */
double RobotRunner::sitPos[12] = {
                        -0.026550, -1.262121, 2.557070, 
                         0.026550, -1.262121, 2.557070, 
                        -0.026550, -1.262121, 2.557070, 
                         0.026550, -1.262121, 2.557070
                                         }; 


RobotRunner::RobotRunner(RobotController* robot_ctrl,
                        ros::NodeHandle *nh,
                        RobotType robotType,
                        PeriodicTaskManager* manager, 
                        float period, 
                        std::string name)
        : PeriodicTask(manager, period, name),
          robotType(robotType),
          duration(2*1000), 
          loop_count(0)//TEST
{
     _robot_ctrl = robot_ctrl;

    // kpMat << 70, 0, 0, 0, 180, 0, 0, 0, 300;
    // kpMat << 30, 0, 0, 0, 30, 0, 0, 0, 30;
    // kpMat << 80, 0, 0, 0, 80, 0, 0, 0, 80;
    kpMat << 35, 0, 0, 0, 35, 0, 0, 0, 35;

    // kdMat << 3, 0, 0, 0, 8, 0, 0, 0, 15;
    // kdMat << 3, 0, 0, 0, 8, 0, 0, 0, 8;
    // kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    kdMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;

    if(robotType == RobotType::UNITREE_A1) {
        robotName = "UNITREE_A1";//UNKNOWN
    } else if(robotType == RobotType::UNITREE_ALIENGO) {
        robotName = "aliengo";
    } else {
        assert(false);
    }

    stateEstimatePub = nh->advertise<unitree_legged_msgs::StateEstimate>("/" + robotName + "_gazebo/StateEstimation", 1);

    // sit_flag = false;
}

void RobotRunner::init()
{
    if(robotType == RobotType::UNITREE_A1) {
    _quadruped = buildA1<float>();
    } else if((robotType == RobotType::UNITREE_ALIENGO)) {
        _quadruped = buildAliengo<float>();
    } else {
        assert(false);
    }

    _model = _quadruped.buildModel();

    _legController = new LegController<float>(_quadruped);
    _jpos_initializer = new JPosInitializer<float>(3., 0.002);
    _sit_jpos_initializer = new JPosInitializer<float>(3., 0.002);
    _stateEstimator = new StateEstimatorContainer<float>(vectorNavData, _legController->datas, &_stateEstimate, controlParameters);
    initializeStateEstimator();

    _desiredStateCommand = new DesiredStateCommand<float>();

    _robot_ctrl->_model = &_model;
	_robot_ctrl->_quadruped = &_quadruped;
	_robot_ctrl->_legController = _legController;
  	_robot_ctrl->_stateEstimator = _stateEstimator;
  	_robot_ctrl->_stateEstimate = &_stateEstimate;
	// _robot_ctrl->_visualizationData= visualizationData;
	_robot_ctrl->_robotType = RobotType::MINI_CHEETAH;
	// _robot_ctrl->_driverCommand = driverCommand;
	_robot_ctrl->_controlParameters = controlParameters;
	_robot_ctrl->_desiredStateCommand = _desiredStateCommand;

	 _robot_ctrl->initializeController();
    
}

void RobotRunner::run()
{  
    loop_count++;
    

     //Run State Estimator
    _stateEstimator->run();

    // //Update input to desiredstatecommand
    // updateDesiredStateCommand();


    //Setup Step;
    _legController->updateData(lowState);
	_legController->zeroCommand();

    if(*sit_flag) {
        _sit_jpos_initializer->UpdateSitParam();
        run_sit_jpos();
    } else {
        run_controller();
        // run_jpos();
    }
    
    //DEBUG
    // if(loop_count%100 == 0)
    //     _stateEstimator->print_estimate();
    
    //Finalize Step
    _legController->updateCommand(lowCmd);

    pubStateEstimate();
}

void RobotRunner::initializeStateEstimator() 
{
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);

 
    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
   _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
 
}

// void RobotRunner::updateDesiredStateCommand()
// {
//     _desiredStateCommand->leftAnalogStick[1] = twist->linear.x;
//     _desiredStateCommand->leftAnalogStick[0] = twist->linear.y; 
//     _desiredStateCommand->rightAnalogStick[0] = twist->angular.z;
// }


void RobotRunner::run_jpos()
{
    //Run JPos Initializer 
    if(!_jpos_initializer->IsInitialized(_legController))
	{
        for (int leg = 0; leg < 4; leg++) 
        {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
	}
    else
	{
        for (int leg = 0; leg < 4; leg++) 
        {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
    	for (int leg=0; leg<4; ++leg)
	    	for (int jidx=0; jidx<3; ++jidx)
		    {
		    	_legController->commands[leg].tauFeedForward[jidx] = 0.;
	        	_legController->commands[leg].qDes[jidx] = targetPos[3 * leg + jidx];
	        	_legController->commands[leg].qdDes[jidx] = 0.;
		    }
	}

}

void RobotRunner::run_controller()
{
    //Run JPos Initializer 
    if(!_jpos_initializer->IsInitialized(_legController))
	{
        for (int leg = 0; leg < 4; leg++) 
        {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
	}
    else
	{
        _robot_ctrl->runController();
	}

}

void RobotRunner::balance_standup()
{
    //Run JPos Initializer 
    if(!_jpos_initializer->IsInitialized(_legController))
	{
        for (int leg = 0; leg < 4; leg++) 
        {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
	}
    else
	{
        _robot_ctrl->runController();
	}

        // _robot_ctrl->runController();


}

void RobotRunner::run_sit_jpos()
{
    /* Run JPos Initializer for sitting */
    if(!_sit_jpos_initializer->IsInitialized(_legController))
	{
        for (int leg = 0; leg < 4; leg++) 
        {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
	}
    else
	{
        for (int leg = 0; leg < 4; leg++) 
        {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
    	for (int leg=0; leg<4; ++leg)
	    	for (int jidx=0; jidx<3; ++jidx)
		    {
		    	_legController->commands[leg].tauFeedForward[jidx] = 0.;
	        	_legController->commands[leg].qDes[jidx] = sitPos[3 * leg + jidx];
	        	_legController->commands[leg].qdDes[jidx] = 0.;
		    }
	}
}

void RobotRunner::pubStateEstimate()
{
    /* Set Contact Estimate */
    _stateEstimateROS.contactEstimate[0] = _stateEstimate.contactEstimate[0];
    _stateEstimateROS.contactEstimate[1] = _stateEstimate.contactEstimate[1];
    _stateEstimateROS.contactEstimate[2] = _stateEstimate.contactEstimate[2];
    _stateEstimateROS.contactEstimate[3] = _stateEstimate.contactEstimate[3];

    /* Set CoM Position */
    _stateEstimateROS.comPosition.x = _stateEstimate.position[0];
    _stateEstimateROS.comPosition.y = _stateEstimate.position[1];
    _stateEstimateROS.comPosition.z = _stateEstimate.position[2];

    /* Set CoM Velocity */
    _stateEstimateROS.comVelocity.x = _stateEstimate.vBody[0];
    _stateEstimateROS.comVelocity.y = _stateEstimate.vBody[1];
    _stateEstimateROS.comVelocity.z = _stateEstimate.vBody[2];

    /* Set Feet Position
    *
    * FRONT
    * 1 0   RIGHT
    * 3 2
    * BACK */
    for( int leg = 0; leg < 4; leg++){
        _stateEstimateROS.feetPosition2Body[leg].x = _legController->datas[leg].p[0];
        _stateEstimateROS.feetPosition2Body[leg].y = _legController->datas[leg].p[1];
        _stateEstimateROS.feetPosition2Body[leg].z = _legController->datas[leg].p[2];
    }

    /* Set Feet Velocity */
    for( int leg = 0; leg < 4; leg++){
        _stateEstimateROS.feetVelocity2Body[leg].x = _legController->datas[leg].v[0];
        _stateEstimateROS.feetVelocity2Body[leg].y = _legController->datas[leg].v[1];
        _stateEstimateROS.feetVelocity2Body[leg].z = _legController->datas[leg].v[2];
    }

    /* Set Body Orientation */
    for( int i = 0; i < 4; i++){
        _stateEstimateROS.orientation[i] = _stateEstimate.orientation[i]; 
    } 

    /* Set rpy */
    for( int i = 0; i < 3; i++){
        _stateEstimateROS.rpy[i] = _stateEstimate.rpy[i];
    }

    /* Set Angular Vel */
    for( int i = 0; i < 3; i++){
        _stateEstimateROS.omegaBody[i] = _stateEstimate.omegaBody[i];
    }

    /* Publish to ROS-Topic */
    stateEstimatePub.publish(_stateEstimateROS);
}

void RobotRunner::cleanup()
{}

RobotRunner::~RobotRunner()
{
    delete _legController;
    delete _stateEstimator;
    delete _jpos_initializer;
    delete _desiredStateCommand;
    printf("Deallocating Robot Runner");
}