#pragma once

#include <ros/ros.h>
#include <memory>
#include <unordered_map>
#include <stdexcept>
#include <chrono>
#include <pthread.h>
#include <mutex>
#include <thread>

// #include <ocs2_tracking_control/Controller.h>
// #include <ocs2_core/Types.h>

#include <std_msgs/String.h>
// #include <ocs2_quadruped_msgs/LowState.h>
// #include <ocs2_quadruped_msgs/FootForces.h>
// #include "ocs2_quadruped_msgs/RobotState.h"
#include <std_msgs/Float32MultiArray.h>

// #include <unitree/robot/channel/channel_publisher.hpp>
// #include <unitree/robot/channel/channel_subscriber.hpp>
// #include <unitree/idl/go2/LowState_.hpp>
// #include <unitree/idl/go2/LowCmd_.hpp>
// #include <unitree/idl/go2/SportModeState_.hpp>
// #include <unitree/common/time/time_tool.hpp>
// #include <unitree/common/thread/thread.hpp>

#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>


// using namespace unitree::common;
// using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

// namespace ocs2_robot_driver {

// using namespace ocs2;

// Mapping from joint names to motorCmd index
const std::map<std::string, int> joint_to_motor_index = {
    {"RF_HAA", 0}, {"RF_HFE", 1}, {"RF_KFE", 2},
    {"LF_HAA", 3}, {"LF_HFE", 4}, {"LF_KFE", 5},
    {"RH_HAA", 6}, {"RH_HFE", 7}, {"RH_KFE", 8},
    {"LH_HAA", 9}, {"LH_HFE", 10}, {"LH_KFE", 11}
};

// Mapping from ocs2 feet notation to unitree feet notation
const std::map<int, int> ocs2_to_unitree_feet_index = {
    {0, 1}, // ocs2 LF -> unitree FL
    {1, 0}, // ocs2 RF -> unitree FR
    {2, 3}, // ocs2 LH -> unitree RL
    {3, 2}  // ocs2 RH -> unitree RR
};

const std::vector<std::string> jointNames_ = {"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                   "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

// uint32_t crc32_core(uint32_t *ptr, uint32_t len)
// {
//     unsigned int xbit = 0;
//     unsigned int data = 0;
//     unsigned int CRC32 = 0xFFFFFFFF;
//     const unsigned int dwPolynomial = 0x04c11db7;

//     for (unsigned int i = 0; i < len; i++)
//     {
//         xbit = 1 << 31;
//         data = ptr[i];
//         for (unsigned int bits = 0; bits < 32; bits++)
//         {
//             if (CRC32 & 0x80000000)
//             {
//                 CRC32 <<= 1;
//                 CRC32 ^= dwPolynomial;
//             }
//             else
//             {
//                 CRC32 <<= 1;
//             }

//             if (data & xbit)
//                 CRC32 ^= dwPolynomial;
//             xbit >>= 1;
//         }
//     }

//     return CRC32;
// }

class MujocoCommunicationChannel {
   public:
   MujocoCommunicationChannel(
    ros::NodeHandle *nh, 
    unitree_legged_msgs::LowCmd* lowCmdPtr,  
    unitree_legged_msgs::LowState* lowStatePtr
);


    //void init();

    //void step(const double dt);

    //void addController(std::unique_ptr<ocs2_tracking_control::Controller> controllerPtr);

    // void pubLowState();
    void pubLowCmd();
    void updatelowState();
    void updatelowCmd();
    // void LowStateMessageHandler(const void *messages);
    void LowStateCallback(const unitree_legged_msgs::LowState::ConstPtr& msg);
    void InitLowCmd();
    // void LowCmdWrite();
    void sendServoCmd();


   private:
    // unitree_go::msg::dds_::LowCmd_ low_cmd{};     // default init
    // unitree_go::msg::dds_::LowState_ low_state{}; // default init
    
    /* Communication variables */ 
    unitree_legged_msgs::LowCmd low_cmd;//publish to ros topic
    unitree_legged_msgs::LowState low_state;//subscribe to ros topic
    unitree_legged_msgs::LowCmd* _lowCmd = nullptr;//send in from outside
    unitree_legged_msgs::LowState* _lowState = nullptr;////send in from outside controller

    /*publisher*/
    ros::Publisher lowState_pub;
    ros::Publisher lowCmd_pub;

    /*subscriber*/
    ros::Subscriber lowState_sub;
    
    // /*LowCmd write thread*/
    // ThreadPtr lowCmdWriteThreadPtr;

    // ocs2_quadruped_msgs::LowState lowStateMsg_;
    // ocs2_quadruped_msgs::FootForces footForcesArray;
    std::mutex lowStateMutex_;
    float contactThreshold_[4] = {30.0, 30.0, 30.0, 30.0};
    double pubLoopIter = 0;
    double yaw_last = 0.0;

    // uint32_t initTime_;
};

// }  // namespace ocs2_tracking_control