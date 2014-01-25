#ifndef GATHERER_ARM_H
#define GATHERER_ARM_H

#include "wpilib.h"

class GathererArm : public PIDOutput {
	Talon *leftArm, *rightArm;
public:
	GathererArm(Talon* leftArm, Talon* rightArm);
	virtual void PIDWrite(float output);
};
#endif
