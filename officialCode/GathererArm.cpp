#include "GathererArm.h"

GathererArm::GathererArm(Talon* leftArm, Talon* rightArm) :
	leftArm(leftArm), rightArm(rightArm)
{
}
void GathererArm::PIDWrite(float output) {
	leftArm->PIDWrite(output);
	rightArm->PIDWrite(output);
}
