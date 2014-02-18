#include "Gatherer.h"
#include "WPILib.h"

Gatherer::Gatherer(Talon *roller, AnalogChannel *armAngle, PIDController *armController):
		roller(roller), armAngle(armAngle), armController(armController), pidEnabled(false)
{
	armAngle->SetVoltageForPID(true);
	armController->SetOutputRange(-1,1);
}
void Gatherer::rollerControl(double rollerSpeed) {
	roller->Set(rollerSpeed);
}
void Gatherer::setArmAngle(double value) {
	//some restrictions on value might be needed
	if(pidEnabled)
		armController->SetSetpoint(value);
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
