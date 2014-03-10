#include "Shooter.h"
#include "WPILib.h"


Shooter::Shooter(SpeedController* winch, AnalogChannel* shooterPot, DoubleSolenoid * trigger):
winch(winch), shooterPot(shooterPot), trigger(trigger), state(DRAWING_BACK), 
highThreshold(4.0), lowThreshold(1.0), POT_MIN(0.8), autoLoad(false)
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

void Shooter::runShooter(bool shootButton) {
	switch (state) {
	case DRAWING_BACK:
		runWinch();
		engageWinch();
		if(shooterPot->GetVoltage() < lowThreshold)state = DRAWN_BACK;
		break;
	case DRAWN_BACK:
		stopWinch();
		engageWinch();
		if(shootButton)state = SHOOTING;
		break;
	case SHOOTING:
		stopWinch();
		releaseWinch();
		if(shooterPot->GetVoltage() > highThreshold)state = DRAWING_BACK;
		break;
	}
}
bool Shooter::isWinchEngaged() {
	return trigger->Get() == DoubleSolenoid::kForward;
}
bool Shooter::isReadyToShoot() {
	return state == DRAWN_BACK;
}
