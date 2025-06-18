#include "MujocoCommunicationChannel.h"

MujocoCommunicationChannel::MujocoCommunicationChannel(
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
    // 创建 teleop_twist 订阅者
    teleop_twist.reset(new ChannelSubscriber<unitree_go::msg::dds_::Twist>("/cmd_vel"));
    teleop_twist->InitChannel(std::bind(&MujocoCommunicationChannel::teleopCallback, this, std::placeholders::_1), 1);

    // 创建 imu_sub 订阅者
    imu_sub.reset(new ChannelSubscriber<unitree_go::msg::dds_::Imu>("/trunk_imu"));
    imu_sub->InitChannel(std::bind(&MujocoCommunicationChannel::imuCallback, this, std::placeholders::_1), 1);

    // 创建 footForce_sub 订阅者
    footForce_sub[0].reset(new ChannelSubscriber<unitree_go::msg::dds_::Wrench>("/visual/FR_foot_contact/the_force"));
    footForce_sub[0]->InitChannel(std::bind(&MujocoCommunicationChannel::FRfootCallback, this, std::placeholders::_1), 1);

    footForce_sub[1].reset(new ChannelSubscriber<unitree_go::msg::dds_::Wrench>("/visual/FL_foot_contact/the_force"));
    footForce_sub[1]->InitChannel(std::bind(&MujocoCommunicationChannel::FLfootCallback, this, std::placeholders::_1), 1);

    footForce_sub[2].reset(new ChannelSubscriber<unitree_go::msg::dds_::Wrench>("/visual/RR_foot_contact/the_force"));
    footForce_sub[2]->InitChannel(std::bind(&MujocoCommunicationChannel::RRfootCallback, this, std::placeholders::_1), 1);

    footForce_sub[3].reset(new ChannelSubscriber<unitree_go::msg::dds_::Wrench>("/visual/RL_foot_contact/the_force"));
    footForce_sub[3]->InitChannel(std::bind(&MujocoCommunicationChannel::RLfootCallback, this, std::placeholders::_1), 1);

    // 创建 servo_sub 订阅者
    servo_sub[0].reset(new ChannelSubscriber<unitree_go::msg::dds_::JointControllerState>("/" + robotName + "_gazebo/FR_hip_controller/state"));
    servo_sub[0]->InitChannel(std::bind(&MujocoCommunicationChannel::FRhipCallback, this, std::placeholders::_1), 1);

    servo_sub[1].reset(new ChannelSubscriber<unitree_go::msg::dds_::JointControllerState>("/" + robotName + "_gazebo/FR_thigh_controller/state"));
    servo_sub[1]->InitChannel(std::bind(&MujocoCommunicationChannel::FRthighCallback, this, std::placeholders::_1), 1);

    servo_sub[2].reset(new ChannelSubscriber<unitree_go::msg::dds_::JointControllerState>("/" + robotName + "_gazebo/FR_calf_controller/state"));
    servo_sub[2]->InitChannel(std::bind(&MujocoCommunicationChannel::FRcalfCallback, this, std::placeholders::_1), 1);

    servo_sub[3].reset(new ChannelSubscriber<unitree_go::msg::dds_::JointControllerState>("/" + robotName + "_gazebo/FL_hip_controller/state"));
    servo_sub[3]->InitChannel(std::bind(&MujocoCommunicationChannel::FLhipCallback, this, std::placeholders::_1), 1);

    servo_sub[4].reset(new ChannelSubscriber<unitree_go::msg::dds_::JointControllerState>("/" + robotName + "_gazebo/FL_thigh_controller/state"));
    servo_sub[4]->InitChannel(std::bind(&MujocoCommunicationChannel::FLthighCallback, this, std::placeholders::_1), 1);

    servo_sub[5].reset(new ChannelSubscriber<unitree_go::msg::dds_::JointControllerState>("/" + robotName + "_gazebo/FL_calf_controller/state"));
    servo_sub[5]->InitChannel(std::bind(&MujocoCommunicationChannel::FLcalfCallback, this, std::placeholders::_1), 1);

    servo_sub[6].reset(new ChannelSubscriber<unitree_go::msg::dds_::JointControllerState>("/" + robotName + "_gazebo/RR_hip_controller/state"));
    servo_sub[6]->InitChannel(std::bind(&MujocoCommunicationChannel::RRhipCallback, this, std::placeholders::_1), 1);

    servo_sub[7].reset(new ChannelSubscriber<unitree_go::msg::dds_::JointControllerState>("/" + robotName + "_gazebo/RR_thigh_controller/state"));
    servo_sub[7]->InitChannel(std::bind(&MujocoCommunicationChannel::RRthighCallback, this, std::placeholders::_1), 1);

    servo_sub[8].reset(new ChannelSubscriber<unitree_go::msg::dds_::JointControllerState>("/" + robotName + "_gazebo/RR_calf_controller/state"));
    servo_sub[8]->InitChannel(std::bind(&MujocoCommunicationChannel::RRcalfCallback, this, std::placeholders::_1), 1);

    servo_sub[9].reset(new ChannelSubscriber<unitree_go::msg::dds_::JointControllerState>("/" + robotName + "_gazebo/RL_hip_controller/state"));
    servo_sub[9]->InitChannel(std::bind(&MujocoCommunicationChannel::RLhipCallback, this, std::placeholders::_1), 1);

    servo_sub[10].reset(new ChannelSubscriber<unitree_go::msg::dds_::JointControllerState>("/" + robotName + "_gazebo/RL_thigh_controller/state"));
    servo_sub[10]->InitChannel(std::bind(&MujocoCommunicationChannel::RLthighCallback, this, std::placeholders::_1), 1);

    servo_sub[11].reset(new ChannelSubscriber<unitree_go::msg::dds_::JointControllerState>("/" + robotName + "_gazebo/RL_calf_controller/state"));
    servo_sub[11]->InitChannel(std::bind(&MujocoCommunicationChannel::RLcalfCallback, this, std::placeholders::_1), 1);


    


    // Publisher Initialization    
    // 创建 servo_pub 发布者
    servo_pub[0].reset(new ChannelPublisher<unitree_go::msg::dds_::MotorCmd>("/" + robotName + "_gazebo/FR_hip_controller/command"));
    servo_pub[0]->InitChannel();

    servo_pub[1].reset(new ChannelPublisher<unitree_go::msg::dds_::MotorCmd>("/" + robotName + "_gazebo/FR_thigh_controller/command"));
    servo_pub[1]->InitChannel();

    servo_pub[2].reset(new ChannelPublisher<unitree_go::msg::dds_::MotorCmd>("/" + robotName + "_gazebo/FR_calf_controller/command"));
    servo_pub[2]->InitChannel();

    servo_pub[3].reset(new ChannelPublisher<unitree_go::msg::dds_::MotorCmd>("/" + robotName + "_gazebo/FL_hip_controller/command"));
    servo_pub[3]->InitChannel();

    servo_pub[4].reset(new ChannelPublisher<unitree_go::msg::dds_::MotorCmd>("/" + robotName + "_gazebo/FL_thigh_controller/command"));
    servo_pub[4]->InitChannel();

    servo_pub[5].reset(new ChannelPublisher<unitree_go::msg::dds_::MotorCmd>("/" + robotName + "_gazebo/FL_calf_controller/command"));
    servo_pub[5]->InitChannel();

    servo_pub[6].reset(new ChannelPublisher<unitree_go::msg::dds_::MotorCmd>("/" + robotName + "_gazebo/RR_hip_controller/command"));
    servo_pub[6]->InitChannel();

    servo_pub[7].reset(new ChannelPublisher<unitree_go::msg::dds_::MotorCmd>("/" + robotName + "_gazebo/RR_thigh_controller/command"));
    servo_pub[7]->InitChannel();

    servo_pub[8].reset(new ChannelPublisher<unitree_go::msg::dds_::MotorCmd>("/" + robotName + "_gazebo/RR_calf_controller/command"));
    servo_pub[8]->InitChannel();

    servo_pub[9].reset(new ChannelPublisher<unitree_go::msg::dds_::MotorCmd>("/" + robotName + "_gazebo/RL_hip_controller/command"));
    servo_pub[9]->InitChannel();

    servo_pub[10].reset(new ChannelPublisher<unitree_go::msg::dds_::MotorCmd>("/" + robotName + "_gazebo/RL_thigh_controller/command"));
    servo_pub[10]->InitChannel();

    servo_pub[11].reset(new ChannelPublisher<unitree_go::msg::dds_::MotorCmd>("/" + robotName + "_gazebo/RL_calf_controller/command"));
    servo_pub[11]->InitChannel();

    // 创建 lowState_pub 发布者
    lowState_pub.reset(new ChannelPublisher<unitree_go::msg::dds_::LowState>("/" + robotName + "_gazebo/lowState/state"));
    lowState_pub->InitChannel();

}

void MujocoCommunicationChannel::sendServoCmd()
{
    for(size_t m=0; m < 12; m++)
        servo_pub[m].publish(_lowCmd->motorCmd[m]);

    pubLowState();
    
    // //Upadate Frequency - 1Khz
    // usleep(1000); 
}

void MujocoCommunicationChannel::pubLowState()
{
    lowState_pub.publish(*_lowState);
}

// Call Back functions 
void MujocoCommunicationChannel::teleopCallback(const geometry_msgs::Twist& msg)
{
    _twist->linear.x = msg.linear.x;
    _twist->linear.y = msg.linear.y;
    _twist->angular.z = msg.angular.z;
}

void MujocoCommunicationChannel::imuCallback(const sensor_msgs::Imu & msg)
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

void MujocoCommunicationChannel::FRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[0].mode = msg.mode;
    _lowState->motorState[0].q = msg.q;
    _lowState->motorState[0].dq = msg.dq;
    _lowState->motorState[0].tauEst = msg.tauEst;
}

void MujocoCommunicationChannel::FRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[1].mode = msg.mode;
    _lowState->motorState[1].q = msg.q;
    _lowState->motorState[1].dq = msg.dq;
    _lowState->motorState[1].tauEst = msg.tauEst;
}

void MujocoCommunicationChannel::FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[2].mode = msg.mode;
    _lowState->motorState[2].q = msg.q;
    _lowState->motorState[2].dq = msg.dq;
    _lowState->motorState[2].tauEst = msg.tauEst;
}

void MujocoCommunicationChannel::FLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[3].mode = msg.mode;
    _lowState->motorState[3].q = msg.q;
    _lowState->motorState[3].dq = msg.dq;
    _lowState->motorState[3].tauEst = msg.tauEst;
}

void MujocoCommunicationChannel::FLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[4].mode = msg.mode;
    _lowState->motorState[4].q = msg.q;
    _lowState->motorState[4].dq = msg.dq;
    _lowState->motorState[4].tauEst = msg.tauEst;
}

void MujocoCommunicationChannel::FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[5].mode = msg.mode;
    _lowState->motorState[5].q = msg.q;
    _lowState->motorState[5].dq = msg.dq;
    _lowState->motorState[5].tauEst = msg.tauEst;
}

void MujocoCommunicationChannel::RRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[6].mode = msg.mode;
    _lowState->motorState[6].q = msg.q;
    _lowState->motorState[6].dq = msg.dq;
    _lowState->motorState[6].tauEst = msg.tauEst;
}

void MujocoCommunicationChannel::RRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[7].mode = msg.mode;
    _lowState->motorState[7].q = msg.q;
    _lowState->motorState[7].dq = msg.dq;
    _lowState->motorState[7].tauEst = msg.tauEst;
}

void MujocoCommunicationChannel::RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[8].mode = msg.mode;
    _lowState->motorState[8].q = msg.q;
    _lowState->motorState[8].dq = msg.dq;
    _lowState->motorState[8].tauEst = msg.tauEst;
}

void MujocoCommunicationChannel::RLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[9].mode = msg.mode;
    _lowState->motorState[9].q = msg.q;
    _lowState->motorState[9].dq = msg.dq;
    _lowState->motorState[9].tauEst = msg.tauEst;
}

void MujocoCommunicationChannel::RLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[10].mode = msg.mode;
    _lowState->motorState[10].q = msg.q;
    _lowState->motorState[10].dq = msg.dq;
    _lowState->motorState[10].tauEst = msg.tauEst;
}

void MujocoCommunicationChannel::RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState->motorState[11].mode = msg.mode;
    _lowState->motorState[11].q = msg.q;
    _lowState->motorState[11].dq = msg.dq;
    _lowState->motorState[11].tauEst = msg.tauEst;
}

void MujocoCommunicationChannel::FRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    _lowState->eeForce[0].x = msg.wrench.force.x;
    _lowState->eeForce[0].y = msg.wrench.force.y;
    _lowState->eeForce[0].z = msg.wrench.force.z;
    _lowState->footForce[0] = msg.wrench.force.z;
}

void MujocoCommunicationChannel::FLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    _lowState->eeForce[1].x = msg.wrench.force.x;
    _lowState->eeForce[1].y = msg.wrench.force.y;
    _lowState->eeForce[1].z = msg.wrench.force.z;
    _lowState->footForce[1] = msg.wrench.force.z;
}

void MujocoCommunicationChannel::RRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    _lowState->eeForce[2].x = msg.wrench.force.x;
    _lowState->eeForce[2].y = msg.wrench.force.y;
    _lowState->eeForce[2].z = msg.wrench.force.z;
    _lowState->footForce[2] = msg.wrench.force.z;
}

void MujocoCommunicationChannel::RLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    _lowState->eeForce[3].x = msg.wrench.force.x;
    _lowState->eeForce[3].y = msg.wrench.force.y;
    _lowState->eeForce[3].z = msg.wrench.force.z;
    _lowState->footForce[3] = msg.wrench.force.z;
}

MujocoCommunicationChannel::~MujocoCommunicationChannel()
{
    std::cout << "Deallocating Mujoco Communication Channel\n";
}
