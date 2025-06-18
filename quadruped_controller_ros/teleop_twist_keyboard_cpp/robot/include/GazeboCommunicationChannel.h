#ifndef _PROJECT_GAZEBO_COMMUNICATION_CHANNEL_
#define _PROJECT_GAZEBO_COMMUNICATION_CHANNEL_

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

class GazeboCommunicationChannel
{
public:
	GazeboCommunicationChannel(geometry_msgs::Twist* lowCmdPtr, gazebo_msgs::ModelState* _lowStatePtr)
	: 
	_lowCmd(lowCmdPtr),
	_lowState(_lowStatePtr)
	{
		robot_sub = nm.subscribe("/gazebo/model_states", 1, &GazeboCommunicationChannel::modelStateCallBack, this);
		robot_pub = nm.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	}

	void sendTeleopCmd();
	void modelStateCallBack(const gazebo_msgs::ModelStates& msg);


private:

	ros::NodeHandle nm;
	ros::Subscriber robot_sub;
	ros::Publisher robot_pub;
	geometry_msgs::Twist* _lowCmd;
	gazebo_msgs::ModelState* _lowState;

	//DEBUG
	// int count;

};

#endif
