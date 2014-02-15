#include "Shooter.h"
#include "WPILib.h"


Shooter::Shooter(SpeedController* winch, AnalogChannel* shooterPot, DoubleSolenoid * trigger):
winch(winch), shooterPot(shooterPot), trigger(trigger)
{
	trigger->Set(DoubleSolenoid::kForward);
}

void Shooter::runWinch() {
	winch->Set(1);
}

void Shooter::stopWinch() {
	winch->Set(0);
}

void Shooter:: fireTrigger() {

		trigger->Set(DoubleSolenoid::kForward);
}

void Shooter:: retractTrigger() {
	trigger->Set(DoubleSolenoid::kReverse);

}
