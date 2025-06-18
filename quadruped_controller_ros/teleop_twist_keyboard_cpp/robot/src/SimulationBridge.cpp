#include "SimulationBridge.h"

void SimulationBridge::init()
{
	_gazeboCommunicationChannel = new GazeboCommunicationChannel(&_lowCmd, &_lowState);
	// spinner.start();
	usleep(300000);
}

void SimulationBridge::run()
{
	while(ros::ok())
	{
		//print lowstate
		printf("Position x: %f\n", _lowState.pose.position.x);
		usleep(1000000);
		ros::spinOnce();
	}
}

int main(int argc, char **argv)
{
	// initialize the node
	ros::init(argc, argv, "turtlebot_controller");

	// initialize the Simulaiton bridge
	SimulationBridge sb;
	sb.init();
	sb.run();

	return 0;
}