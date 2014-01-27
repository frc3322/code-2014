#include "Gatherer.h"
#include "WPILib.h"

Gatherer::Gatherer(Talon *rollerLeft, Talon *rollerRight,
		AnalogChannel *armAngle, PIDController *armControler):
		leftRoller(rollerLeft), rightRoller(rollerRight), 
		armAngle(armAngle), armController(armController)
{
	//armController.Enable();
	//what should default setpoint be???
}
void Gatherer::rollerControl(double rollerSpeed) {
	rightRoller->Set(rollerSpeed);
	leftRoller->Set(rollerSpeed);
}
void Gatherer::setArmAngle(double value) {
	//some restrictions on value maight be needed
	//armController.SetSetpoint(value);
}
void Gatherer::moveArmForward(double value) {
	//get current angle then set arm angle to current angle + increment
}
void Gatherer::moveArmBackward(double value) {
}
