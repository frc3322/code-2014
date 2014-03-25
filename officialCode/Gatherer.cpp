#include "Gatherer.h"
#include "WPILib.h"

Gatherer::Gatherer(Talon *roller, AnalogChannel *armAngle, PIDController *armController, double offset):
		roller(roller), armAngle(armAngle), armController(armController), pidEnabled(false),offset(offset),
		#if ROBOT == COMP
		FORWARD_POSITION(3.73), BACKWARD_POSITION(2.75), UP_POSITION(3.1), DOWN_POSITION(4.0)
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
