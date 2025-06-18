#include "MujocoCommunicationChannel_2.h"

#include <Eigen/Geometry>
// #include <ocs2_anymal_commands/TerrainAdaptation.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// namespace ocs2_robot_driver {

    MujocoCommunicationChannel::MujocoCommunicationChannel
    (
        ros::NodeHandle *nh, 
        unitree_legged_msgs::LowCmd* lowCmdPtr,   // External controller's LowCmd pointer
        unitree_legged_msgs::LowState* lowStatePtr
    )
        : _lowCmd(lowCmdPtr),  // Store the command pointer
          _lowState(lowStatePtr)
        
{
    // // Unitree legged sdk initialization
    // InitLowCmd();

    /*create publisher*/
    lowCmd_pub = nh->advertise<unitree_legged_msgs::LowCmd>("/go1/_gazebo/lowCmd/command", 1);
    
    /*create lowstate subscriber*/
    lowState_sub = nh->subscribe("/go1/_gazebo/lowState/state", 1, &MujocoCommunicationChannel::LowStateCallback, this);

    // initTime_ = ros::Time::now().toSec();

    // // Initialize low state message
    // lowState_pub = nh->advertise<unitree_legged_msgs::LowState>("/go1/_gazebo/lowState/state", 1);
}

// void MujocoCommunicationChannel::InitLowCmd()
// {
//     low_cmd.levelFlag = 0xFF;
//     low_cmd.commVersion = 0x0001;
//     low_cmd.SN = 0;
//     low_cmd.bandWidth = 0xFF;

//     for (int i = 0; i < 20; i++)
//     {
//         low_cmd.motorCmd[i].mode = 0x01; // motor switch to servo (PMSM) mode
//         low_cmd.motorCmd[i].q = PosStopF;
//         low_cmd.motorCmd[i].Kp = 0.0f;
//         low_cmd.motorCmd[i].dq = VelStopF;
//         low_cmd.motorCmd[i].Kd = 0.0f;
//         low_cmd.motorCmd[i].tau = 0.0f;
//     }
// }

void MujocoCommunicationChannel::LowStateCallback(const unitree_legged_msgs::LowState::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(lowStateMutex_);    
    low_state = *msg; 
}

void MujocoCommunicationChannel::updatelowCmd()
{
    // 遍历 12 个电机，填充 low_cmd.motor_cmd()
    for(size_t m = 0; m < 12; m++) 
    {
        low_cmd.motorCmd[m].q  = static_cast<float>(_lowCmd->motorCmd[m].q);
        low_cmd.motorCmd[m].dq = static_cast<float>(_lowCmd->motorCmd[m].dq);
        low_cmd.motorCmd[m].tau = static_cast<float>(_lowCmd->motorCmd[m].tau);
        low_cmd.motorCmd[m].Kp  = static_cast<float>(_lowCmd->motorCmd[m].Kp);
        low_cmd.motorCmd[m].Kd  = static_cast<float>(_lowCmd->motorCmd[m].Kd);
    }
}

void MujocoCommunicationChannel::sendServoCmd()
{
    pubLowCmd();
    updatelowState();
    // 发布 lowState 以确保状态同步
    // pubLowState();
}

void MujocoCommunicationChannel::pubLowCmd()
{
    // 先更新电机状态
    updatelowCmd();
    // 发送命令
    lowCmd_pub.publish(low_cmd);
}

// void MujocoCommunicationChannel::pubLowState()
// {
//     // 先更新电机状态
//     updatelowState(msg);
//     // // 发布 low state
//     // lowState_pub.publish(*_lowState);
// }

// 更新电机状态
void MujocoCommunicationChannel::updatelowState()
{
    std::lock_guard<std::mutex> lock(lowStateMutex_);
    for (size_t m = 0; m < 12; m++) 
    {
        _lowState->motorState[m].mode = low_state.motorState[m].mode;
        _lowState->motorState[m].q = low_state.motorState[m].q;
        _lowState->motorState[m].dq = low_state.motorState[m].dq;
        _lowState->motorState[m].tauEst = low_state.motorState[m].tauEst;
    }
}
