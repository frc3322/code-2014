#ifndef TRIPLE_SPEED_CONTROLLER_H
#define TRIPLE_SPEED_CONTROLLER_H
#include "wpilib.h"
#include <limits>	//for nan

class TripleSpeedController : public SpeedController{
	SpeedController* controllers[3];
public:
	TripleSpeedController(SpeedController* c1, SpeedController* c2, SpeedController* c3);
	virtual void Set(float speed, uint8_t syncGroup);
	virtual float Get();	//returns avg
	float Get(unsigned int index);	//returns value for a certain controller or nan if invalid index
	virtual void Disable();
	virtual void PIDWrite(float value);
};
#endif
