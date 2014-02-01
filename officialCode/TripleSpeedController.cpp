#include "TripleSpeedController.h"

TripleSpeedController::TripleSpeedController(SpeedController* c1, SpeedController* c2, SpeedController* c3)
{
	controllers[0] = c1;
	controllers[1] = c2;
	controllers[2] = c3;
}
void TripleSpeedController::Set(float speed, uint8_t syncGroup=0) {
	controllers[0]->Set(speed,syncGroup);
	controllers[1]->Set(speed,syncGroup);
	controllers[2]->Set(speed,syncGroup);
}
float TripleSpeedController::Get() {
	return (controllers[0]->Get() + controllers[1]->Get() + controllers[2]->Get())/3.0;
}
float TripleSpeedController::Get(unsigned int index) {
	if(index > 2)return std::numeric_limits<float>::quiet_NaN();
	return controllers[index]->Get();
}
void TripleSpeedController::Disable() {
	controllers[0]->Disable();
	controllers[1]->Disable();
	controllers[2]->Disable();
}
void TripleSpeedController::PIDWrite(float value) {
	Set(value);
}
