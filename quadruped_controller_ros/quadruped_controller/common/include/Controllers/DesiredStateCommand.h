/*!
 * @file DesiredStateCommand.h
 * @brief Logic to convert a joystick command into a desired trajectory for the robot
 *
 * This will generate a state trajectory which can easily be used for model predictive controllers
 */

/*========================= Gamepad Control ==========================*/
/**
 *
 */
#ifndef DESIRED_STATE_COMMAND_H
#define DESIRED_STATE_COMMAND_H

#include <iostream>
#include "Types/cppTypes.h"

template <typename T>
class DesiredStateCommand 
{

public:	
	Vec2<float> leftAnalogStick;
  	Vec2<float> rightAnalogStick;
	Vec2<float> joystickLeft, joystickRight;
  	bool trigger_pressed = false;
	DesiredStateCommand();
  	void convertToStateCommands();

private:
	const T filter = 0.1;

};

#endif