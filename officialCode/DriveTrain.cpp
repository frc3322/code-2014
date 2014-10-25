#include "DriveTrain.h"
DriveTrain::DriveTrain(SpeedController *left, SpeedController *right,
				DoubleSolenoid *gearShifter,
				Encoder *leftDriveEncoder, Encoder *rightDriveEncoder): 
				RobotDrive(left,right), 
				shifter(gearShifter),
				leftEncoder(leftDriveEncoder), rightEncoder(rightDriveEncoder),
				sampleIndex(0), shiftPoint(60.0), autoShiftEnabled(true),
				shiftHighPoint(3000.0), shiftLowPoint(2000.0),
				shiftHighCounter(0),shiftLowCounter(0), shiftCounterThreshold(3)
{
	for(unsigned int i = 0; i < NUM_SPEED_SAMPLES; i++){
		leftSpeedHistory[i] = 0;
		rightSpeedHistory[i] = 0;
	}
}
bool DriveTrain::isInHighGear() {
	return shifter->Get() == DoubleSolenoid::kReverse;
}
void DriveTrain::toggleShiftGear() {
	shifter->Set(isInHighGear() ? DoubleSolenoid::kForward : DoubleSolenoid::kReverse);
}
void DriveTrain::shiftHighGear() {
	shifter->Set(DoubleSolenoid::kReverse);
}
void DriveTrain::shiftLowGear() {
	shifter->Set(DoubleSolenoid::kForward);
}
void DriveTrain::toogleAutoShiftEnable() {
	autoShiftEnabled = !autoShiftEnabled;
}
void DriveTrain::setAutoShiftEnable(bool value) {
	autoShiftEnabled = value;
}
bool DriveTrain::isAutoShiftEnabled() {
	return autoShiftEnabled;
}
void DriveTrain::takeSpeedSample() {
	leftSpeedHistory[sampleIndex] = leftEncoder->GetRate();
	rightSpeedHistory[sampleIndex++] = rightEncoder->GetRate();
	if(sampleIndex >= NUM_SPEED_SAMPLES) sampleIndex = 0;
}
void DriveTrain::shiftAutomatically() {
	if(!autoShiftEnabled)return;
	//find average speeds for both left and right encoders
	double leftAvg = 0.0, rightAvg = 0.0;
	for(unsigned int i = 0; i < NUM_SPEED_SAMPLES; i++) {
		leftAvg += leftSpeedHistory[i];
		rightAvg += rightSpeedHistory[i];
	}
	leftAvg /= ((double)NUM_SPEED_SAMPLES);
	rightAvg /= ((double)NUM_SPEED_SAMPLES);
	if(fabs(leftAvg + rightAvg)/2.0 > shiftHighPoint){
		shiftHighCounter++;
		if(shiftHighCounter > shiftCounterThreshold)shiftHighGear();
		shiftLowCounter = 0;
	}else if(fabs(leftAvg + rightAvg)/2.0 < shiftLowPoint) {
		shiftLowCounter++;
		if(shiftLowCounter > shiftCounterThreshold)shiftLowGear();
		shiftHighCounter = 0;
	}else {
		shiftHighCounter = shiftLowCounter = 0;
	}
}
bool DriveTrain::isTurning() {
	//if encoders have opposite signs and
	return leftEncoder->GetRate() * rightEncoder->GetRate() < 0 ;
}
