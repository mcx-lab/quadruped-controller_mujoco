#include "GazeboCommunicationChannel.h"

GazeboCommunicationChannel::GazeboCommunicationChannel(
    RobotType robotType,
    ros::NodeHandle *nh, 
    unitree_legged_msgs::LowCmd* lowCmdPtr, 
    unitree_legged_msgs::LowState* lowStatePtr,
    geometry_msgs::Twist* twistPtr)
    :_lowCmd(lowCmdPtr),
     _lowState(lowStatePtr),
     _twist(twistPtr),
     robotType(robotType)
{
    
    if(robotType == RobotType::UNITREE_A1) {
        robotName = "a1";
    } else if(robotType == RobotType::UNITREE_ALIENGO) {
        robotName = "aliengo";
    } else {
        assert(false);
    }

    // Subscriber Initialization
    teleop_twist = nh->subscribe("/cmd_vel",1, &GazeboCommunicationChannel::teleopCallback, this);
    imu_sub = nh->subscribe("/trunk_imu", 1, &GazeboCommunicationChannel::imuCallback, this);
    footForce_sub[0] = nh->subscribe("/visual/FR_foot_contact/the_force", 1, &GazeboCommunicationChannel::FRfootCallback, this);
    footForce_sub[1] = nh->subscribe("/visual/FL_foot_contact/the_force", 1, &GazeboCommunicationChannel::FLfootCallback, this);
    footForce_sub[2] = nh->subscribe("/visual/RR_foot_contact/the_force", 1, &GazeboCommunicationChannel::RRfootCallback, this);
    footForce_sub[3] = nh->subscribe("/visual/RL_foot_contact/the_force", 1, &GazeboCommunicationChannel::RLfootCallback, this);
    servo_sub[0] = nh->subscribe("/" + robotName + "_gazebo/FR_hip_controller/state", 1, &GazeboCommunicationChannel::FRhipCallback, this);
    servo_sub[1] = nh->subscribe("/" + robotName + "_gazebo/FR_thigh_controller/state", 1, &GazeboCommunicationChannel::FRthighCallback, this);
    servo_sub[2] = nh->subscribe("/" + robotName + "_gazebo/FR_calf_controller/state", 1, &GazeboCommunicationChannel::FRcalfCallback, this);
    servo_sub[3] = nh->subscribe("/" + robotName + "_gazebo/FL_hip_controller/state", 1, &GazeboCommunicationChannel::FLhipCallback, this);
    servo_sub[4] = nh->subscribe("/" + robotName + "_gazebo/FL_thigh_controller/state", 1, &GazeboCommunicationChannel::FLthighCallback, this);
    servo_sub[5] = nh->subscribe("/" + robotName + "_gazebo/FL_calf_controller/state", 1, &GazeboCommunicationChannel::FLcalfCallback, this);
    servo_sub[6] = nh->subscribe("/" + robotName + "_gazebo/RR_hip_controller/state", 1, &GazeboCommunicationChannel::RRhipCallback, this);
    servo_sub[7] = nh->subscribe("/" + robotName + "_gazebo/RR_thigh_controller/state", 1, &GazeboCommunicationChannel::RRthighCallback, this);
    servo_sub[8] = nh->subscribe("/" + robotName + "_gazebo/RR_calf_controller/state", 1, &GazeboCommunicationChannel::RRcalfCallback, this);
    servo_sub[9] = nh->subscribe("/" + robotName + "_gazebo/RL_hip_controller/state", 1, &GazeboCommunicationChannel::RLhipCallback, this);
    servo_sub[10] = nh->subscribe("/" + robotName + "_gazebo/RL_thigh_controller/state", 1, &GazeboCommunicationChannel::RLthighCallback, this);
    servo_sub[11] = nh->subscribe("/" + robotName + "_gazebo/RL_calf_controller/state", 1, &GazeboCommunicationChannel::RLcalfCallback, this);


    // Publisher Initialization    
    servo_pub[0] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robotName + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robotName + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robotName + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robotName + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robotName + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robotName + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robotName + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robotName + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robotName + "_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robotName + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robotName + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robotName + "_gazebo/RL_calf_controller/command", 1);
    lowState_pub = nh->advertise<unitree_legged_msgs::LowState>("/" + robotName + "_gazebo/lowState/state", 1);

}

void GazeboCommunicationChannel::sendServoCmd()
{
    for(size_t m=0; m < 12; m++)
        servo_pub[m].publish(_lowCmd->motorCmd[m]);

    pubLowState();
    
    // //Upadate Frequency - 1Khz
    // usleep(1000); 
}

void GazeboCommunicationChannel::pubLowState()
{
    lowState_pub.publish(*_lowState);
}

// Call Back functions 
void GazeboCommunicationChannel::teleopCallback(const geometry_msgs::Twist& msg)
{
    _twist->linear.x = msg.linear.x;
    _twist->linear.y = msg.linear.y;
    _twist->angular.z = msg.angular.z;
}

void GazeboCommunicationChannel::imuCallback(const sensor_msgs::Imu & msg)
{ 
    _lowState->imu.quaternion[0] = msg.orientation.w;
    _lowState->imu.quaternion[1] = msg.orientation.x;
    _lowState->imu.quaternion[2] = msg.orientation.y;
    _lowState->imu.quaternion[3] = msg.orientation.z;

    _lowState->imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState->imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState->imu.gyroscope[2] = msg.angular_velocity.z;

    _lowState->imu.accelerometer[0] = msg.linear_acceleration.x;
    _lowState->imu.accelerometer[1] = msg.linear_acceleration.y;
    _lowState->imu.accelerometer[2] = msg.linear_acceleration.z;
}

void GazeboCommunicationChannel::FRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[0].mode = msg.mode;
    _lowState->motorState[0].q = msg.q;
    _lowState->motorState[0].dq = msg.dq;
    _lowState->motorState[0].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::FRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[1].mode = msg.mode;
    _lowState->motorState[1].q = msg.q;
    _lowState->motorState[1].dq = msg.dq;
    _lowState->motorState[1].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[2].mode = msg.mode;
    _lowState->motorState[2].q = msg.q;
    _lowState->motorState[2].dq = msg.dq;
    _lowState->motorState[2].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::FLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[3].mode = msg.mode;
    _lowState->motorState[3].q = msg.q;
    _lowState->motorState[3].dq = msg.dq;
    _lowState->motorState[3].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::FLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[4].mode = msg.mode;
    _lowState->motorState[4].q = msg.q;
    _lowState->motorState[4].dq = msg.dq;
    _lowState->motorState[4].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[5].mode = msg.mode;
    _lowState->motorState[5].q = msg.q;
    _lowState->motorState[5].dq = msg.dq;
    _lowState->motorState[5].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::RRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[6].mode = msg.mode;
    _lowState->motorState[6].q = msg.q;
    _lowState->motorState[6].dq = msg.dq;
    _lowState->motorState[6].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::RRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[7].mode = msg.mode;
    _lowState->motorState[7].q = msg.q;
    _lowState->motorState[7].dq = msg.dq;
    _lowState->motorState[7].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[8].mode = msg.mode;
    _lowState->motorState[8].q = msg.q;
    _lowState->motorState[8].dq = msg.dq;
    _lowState->motorState[8].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::RLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[9].mode = msg.mode;
    _lowState->motorState[9].q = msg.q;
    _lowState->motorState[9].dq = msg.dq;
    _lowState->motorState[9].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::RLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[10].mode = msg.mode;
    _lowState->motorState[10].q = msg.q;
    _lowState->motorState[10].dq = msg.dq;
    _lowState->motorState[10].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[11].mode = msg.mode;
    _lowState->motorState[11].q = msg.q;
    _lowState->motorState[11].dq = msg.dq;
    _lowState->motorState[11].tauEst = msg.tauEst;
}

void GazeboCommunicationChannel::FRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    _lowState->eeForce[0].x = msg.wrench.force.x;
    _lowState->eeForce[0].y = msg.wrench.force.y;
    _lowState->eeForce[0].z = msg.wrench.force.z;
    _lowState->footForce[0] = msg.wrench.force.z;
}

void GazeboCommunicationChannel::FLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    _lowState->eeForce[1].x = msg.wrench.force.x;
    _lowState->eeForce[1].y = msg.wrench.force.y;
    _lowState->eeForce[1].z = msg.wrench.force.z;
    _lowState->footForce[1] = msg.wrench.force.z;
}

void GazeboCommunicationChannel::RRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    _lowState->eeForce[2].x = msg.wrench.force.x;
    _lowState->eeForce[2].y = msg.wrench.force.y;
    _lowState->eeForce[2].z = msg.wrench.force.z;
    _lowState->footForce[2] = msg.wrench.force.z;
}

void GazeboCommunicationChannel::RLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    _lowState->eeForce[3].x = msg.wrench.force.x;
    _lowState->eeForce[3].y = msg.wrench.force.y;
    _lowState->eeForce[3].z = msg.wrench.force.z;
    _lowState->footForce[3] = msg.wrench.force.z;
}

GazeboCommunicationChannel::~GazeboCommunicationChannel()
{
    std::cout << "Deallocating Gazebo Communication Channel\n";
}
