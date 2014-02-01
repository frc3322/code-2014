#ifndef SHOOTER_H
#define SHOOTER_H
#include "WPILib.h"

class Shooter {
	SpeedController *winch;
	AnalogChannel *potentiometer;
public:
	Shooter(SpeedController* winch, AnalogChannel* pot);
};
#endif
