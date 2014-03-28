#include "Gatherer.h"
#include "WPILib.h"

Gatherer::Gatherer(Talon *roller, AnalogChannel *armAngle, PIDController *armController, double offset):
		roller(roller), armAngle(armAngle), armController(armController), pidEnabled(false),offset(offset),
		#if ROBOT == COMP
		FORWARD_POSITION(3.97), BACKWARD_POSITION(3.00), UP_POSITION(3.35), DOWN_POSITION(4.26)
		#else
		FORWARD_POSITION(3.4), BACKWARD_POSITION(4.8), UP_POSITION(4.25), DOWN_POSITION(3.0)
#endif
{
	armAngle->SetVoltageForPID(true);
	armController->SetOutputRange(-.4,.4);
	setPIDEnabled(pidEnabled);
}
void Gatherer::rollerControl(double rollerSpeed) {
#if ROBOT == COMP
	roller->Set(rollerSpeed);
#else
	roller->Set(-rollerSpeed);
#endif
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
	if(pidEnabled) {
		armController->Enable();
		this->setArmAngle(this->FORWARD_POSITION);
	}
	else
		armController->Disable();
}
bool Gatherer::isPIDEnabled() {
	return pidEnabled;
}
void Gatherer::togglePIDEnabled() {
	setPIDEnabled(!pidEnabled);
}
