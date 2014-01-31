#ifndef MULTI_MOTOR_SPEED_CONTROLLER_H
#define MULTI_MOTOR_SPEED_CONTROLLER_H
#include "wpilib.h"
#include <limits>	//for nan

class MultiMotorSpeedController : SpeedController{
	SpeedController** controllers;
	unsigned int number;
public:
	MultiMotorSpeedController(SpeedController** controllers, unsigned int number);
	virtual void Set(float speed, uint8_t syncGroup);
	virtual float Get();	//returns avg
	float Get(unsigned int index);	//returns value for a certain controller or nan if invalid index
	virtual void Disable();
};
#endif
