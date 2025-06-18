#include "Controllers/DesiredStateCommand.h"

const Vec2<float> LEFT_STICK_ANALOG{0.0, 0.0};
const Vec2<float> RIGHT_STICK_ANALOG{0.0, 0.0};
template <typename T>
DesiredStateCommand<T>::DesiredStateCommand()
{
  leftAnalogStick.setZero();
  rightAnalogStick.setZero();

  joystickLeft.setZero();
  joystickRight.setZero();

}


template <typename T>
void DesiredStateCommand<T>::convertToStateCommands() 
{
	

	// joystickLeft = LEFT_STICK_ANALOG;
    // joystickRight = RIGHT_STICK_ANALOG;

    joystickLeft[0] *= -1.f;
	joystickRight[0] *= -1.f;

	leftAnalogStick = leftAnalogStick * (T(1) - filter) + joystickLeft * filter;
	rightAnalogStick = rightAnalogStick * (T(1) - filter) + joystickRight * filter;
}

template class DesiredStateCommand<double>;
template class DesiredStateCommand<float>;