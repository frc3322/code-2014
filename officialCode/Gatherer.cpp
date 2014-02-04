#include "Gatherer.h"
#include "WPILib.h"

Gatherer::Gatherer(Talon *roller, AnalogChannel *armAngle, PIDController *armControler):
		roller(roller), armAngle(armAngle), armController(armController)
{
}
void Gatherer::rollerControl(double rollerSpeed) {
	roller->Set(rollerSpeed);
}
void Gatherer::setArmAngle(double value) {
	//some restrictions on value might be needed
	//armController->SetSetpoint(value);
}
void Gatherer::moveArmForward(double increment) {
	//get current angle then set arm angle to current angle + increment
	//armController->SetSetpoint(armController->GetSetpoint() + increment);
}
void Gatherer::moveArmBackward(double increment) {
}
void Gatherer::init() {
	//what should default setpoint be???
	//armController->Enable();
}
