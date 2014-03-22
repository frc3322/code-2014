#include "Shooter.h"
#include "WPILib.h"


Shooter::Shooter(SpeedController* winch, AnalogChannel* shooterPot, DoubleSolenoid * trigger):
winch(winch), shooterPot(shooterPot), trigger(trigger), state(DRAWING_BACK), 
highThreshold(2.4), lowThreshold(1.0), POT_MIN(1.3), autoLoad(false)
{
	trigger->Set(DoubleSolenoid::kForward);
}
void Shooter::runWinch() {
#if ROBOT == COMP
	winch->Set(-1);
#else
	winch->Set(-1);
#endif
}
void Shooter::stopWinch() {
	winch->Set(0);
}
void Shooter::releaseWinch() {
		trigger->Set(DoubleSolenoid::kReverse);
}
void Shooter::engageWinch() {
	trigger->Set(DoubleSolenoid::kForward);
}
void Shooter::toggleAutoLoad() {
	autoLoad = !autoLoad;
}

bool Shooter::isWinchEngaged() {
	return trigger->Get() == DoubleSolenoid::kForward;
}
bool Shooter::isDrawnBack() {
	if(shooterPot->GetVoltage() < POT_MIN){
		return true;
	}
	return false;
}
