#ifndef SHOOTER_H
#define SHOOTER_H
#include "WPILib.h"
#include "AnalogChannel.h"

class Shooter {
	SpeedController *winch;
	AnalogChannel *shooterPot;
	DoubleSolenoid *trigger;
public:
	Shooter(SpeedController* winch, AnalogChannel* shooterPot, DoubleSolenoid * trigger);
	void runWinch();
	void stopWinch();
	void fireTrigger();
	void retractTrigger();

};
#endif
