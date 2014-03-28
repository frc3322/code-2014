#include "Gatherer.h"
#include "WPILib.h"

Gatherer::Gatherer(Talon *roller, AnalogChannel *armAngle, PIDController *armController, double offset):
		roller(roller), armAngle(armAngle), armController(armController), pidEnabled(false),offset(offset),
		#if ROBOT == COMP
		FORWARD_POSITION(3.24), BACKWARD_POSITION(2.24), UP_POSITION(2.58), DOWN_POSITION(3.5)
		#else
		FORWARD_POSITION(3.4), BACKWARD_POSITION(4.8), UP_POSITION(4.25), DOWN_POSITION(3.0)
#endif
{
	armAngle->SetVoltageForPID(true);
	armController->SetOutputRange(-.4,.4);
	setPIDEnabled(pidEnabled);
}
void Gatherer::rollerControl(double rollerSpeed) {
	roller->Set(rollerSpeed);
}
void Gatherer::setArmAngle(double value) {
	//some restrictions on value might be needed
	if(pidEnabled)
		armController->SetSetpoint(offset + value);
}
void Gatherer::setDownPosition(double downPos) {
	offset = downPos - DOWN_POSITION;
}
void Gatherer::setPIDEnabled(bool value) {
	pidEnabled = value;
	if(pidEnabled)
		armController->Enable();
	else
		armController->Disable();
}
bool Gatherer::isPIDEnabled() {
	return pidEnabled;
}
void Gatherer::togglePIDEnabled() {
	setPIDEnabled(!pidEnabled);
}
bool Gatherer::isRolling() {
	if((roller->Get() > .5) || (roller->Get() < -.5)){
		return true;
	}
	else {
		return false;
	}
}
