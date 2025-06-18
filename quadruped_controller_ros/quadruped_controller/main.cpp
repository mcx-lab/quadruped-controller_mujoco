#include "SimulationBridge.h"
#include "MIT_Controller.hpp"
#include "RobotController.h"

int main(int argc, char **argv)
{
   
    ros::init(argc, argv, "mit_controller", ros::init_options::NoSigintHandler);
    SimulationBridge sb(argc, argv, new MIT_Controller());
    sb.run();

    ros::shutdown();
}