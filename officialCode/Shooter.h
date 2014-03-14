#ifndef SHOOTER_H
#define SHOOTER_H
#include "WPILib.h"
#include "AnalogChannel.h"
#include "constants.h"

class Shooter {
	SpeedController *winch;
	AnalogChannel *shooterPot;
	DoubleSolenoid *trigger;
	enum {DRAWN_BACK, DRAWING_BACK, SHOOTING} state;
public:
	double highThreshold;
	double lowThreshold;
	Shooter(SpeedController* winch, AnalogChannel* shooterPot, DoubleSolenoid * trigger);
	void runWinch();
	void stopWinch();
	void releaseWinch();
	void engageWinch();
	void runShooter(bool shootButton);
	bool isReadyToShoot();
	bool isWinchEngaged();
	void toggleAutoLoad();
	double POT_MIN;
	bool autoLoad;
};
#endif
