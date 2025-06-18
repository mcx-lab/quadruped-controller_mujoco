#include "GazeboCommunicationChannel.h"

void GazeboCommunicationChannel::sendTeleopCmd()
{
	robot_pub.publish(*_lowCmd);
	usleep(1000);
}

void GazeboCommunicationChannel::modelStateCallBack(const gazebo_msgs::ModelStates& msg)
{
	_lowState->pose = msg.pose[1];
	ROS_INFO("Inside call modelStateCallBack");
	// x = msg.pose[1].position.x;
	// _lowState->twist = msg.twist.twist;

}
