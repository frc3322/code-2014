#include "Gatherer.h"
#include "WPILib.h"

Gatherer::Gatherer(Talon *rollerLeft, Talon *rollerRight,
		AnalogChannel *armAngle, PIDController *armControler):
		leftRoller(rollerLeft), rightRoller(rollerRight), 
		armAngle(armAngle), armController(armController)
{
}
void Gatherer::rollerControl(double rollerSpeed) {
	rightRoller->Set(rollerSpeed);
	leftRoller->Set(rollerSpeed);
}
void Gatherer::setArmAngle(double value) {
}
