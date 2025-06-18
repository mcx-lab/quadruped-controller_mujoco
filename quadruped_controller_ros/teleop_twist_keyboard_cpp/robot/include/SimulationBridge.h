#ifndef _PROJECT_SIMULATION_BRIDGE_
#define _PROJECT_SIMULATION_BRIDGE_


#include "ros/ros.h"
#include "GazeboCommunicationChannel.h"

class SimulationBridge
{

public:

	SimulationBridge()
	// :spinner(1)
	{}
	void run();
	void init();

	// ros::AsyncSpinner spinner;

private:

	geometry_msgs::Twist _lowCmd;
	gazebo_msgs::ModelState _lowState;

	GazeboCommunicationChannel* _gazeboCommunicationChannel = nullptr;

};

#endif