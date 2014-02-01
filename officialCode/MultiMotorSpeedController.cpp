#include "MultiMotorSpeedController.h"

MultiMotorSpeedController::MultiMotorSpeedController(SpeedController** controllers, unsigned int number):
controllers(controllers), number(number)
{}
void MultiMotorSpeedController::Set(float speed, uint8_t syncGroup=0) {
	for(unsigned int i = 0; i < number; i++)
		controllers[i]->Set(speed,syncGroup);
}
float MultiMotorSpeedController::Get() {
	float avg = 0.0;
	for(unsigned int i = 0; i < number; i++)
		avg += controllers[i]->Get();
	return avg / ((double)number);	
}
float MultiMotorSpeedController::Get(unsigned int index) {
	if(index >= number)return std::numeric_limits<float>::quiet_NaN();
	return controllers[index]->Get();
}
void MultiMotorSpeedController::Disable() {
	for(unsigned int i = 0; i < number; i++)
		controllers[i]->Disable();
}
void MultiMotorSpeedController::PIDWrite(float value) {
	Set(value);
}
