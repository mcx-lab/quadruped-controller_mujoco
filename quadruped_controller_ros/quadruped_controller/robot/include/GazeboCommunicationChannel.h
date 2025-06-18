#ifndef _PROJECT_GAZEBO_COMMUNICATION_CHANNEL_
#define _PROJECT_GAZEBO_COMMUNICATION_CHANNEL_

#include "ros/ros.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include "Types/cppTypes.h"

/* ROS Communication Channel between the controller code and Gazebo
     _lowCmd ptr points to commands to Gazebo
     _lowSate ptr points to sate of the robot */
class GazeboCommunicationChannel
{
private:
    /* Communication variables */ 
    unitree_legged_msgs::LowCmd* _lowCmd = nullptr;
    unitree_legged_msgs::LowState* _lowState = nullptr;

    /* Subscribers and Publishers */
    ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub, teleop_twist;
    ros::Publisher servo_pub[12], lowState_pub;

    /* To Prevent copying of the class */
    GazeboCommunicationChannel(const GazeboCommunicationChannel& );
    GazeboCommunicationChannel& operator=(const GazeboCommunicationChannel&);

    /* Teleop input */
    geometry_msgs::Twist* _twist;
    
    RobotType robotType;
    std::string robotName;

    
public:
    GazeboCommunicationChannel(
        RobotType robotType, 
        ros::NodeHandle *nh, 
        unitree_legged_msgs::LowCmd *lowCmdPtr, 
        unitree_legged_msgs::LowState *lowStatePtr, 
        geometry_msgs::Twist* twistPtr
    );
    ~GazeboCommunicationChannel();

    /* Publisher functions */
    void sendServoCmd();
    void pubLowState();

    /* Subscriber Call-back functions for _lowState */
    void teleopCallback(const geometry_msgs::Twist& msg);
    void imuCallback(const sensor_msgs::Imu & msg);
    void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
    void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
    void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);
    void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
    void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
    void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);
    void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
    void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
    void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);
    void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
    void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
    void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);
    void FRfootCallback(const geometry_msgs::WrenchStamped& msg);
    void FLfootCallback(const geometry_msgs::WrenchStamped& msg);
    void RRfootCallback(const geometry_msgs::WrenchStamped& msg);
    void RLfootCallback(const geometry_msgs::WrenchStamped& msg);
};

#endif
