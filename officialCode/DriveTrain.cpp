#include "DriveTrain.h"

DriveTrain::DriveTrain(SpeedController *frontLeftMotor, SpeedController *rearLeftMotor,
				SpeedController *frontRightMotor, SpeedController *rearRightMotor,
				DoubleSolenoid *gearShifter,
				Encoder *leftDriveEncoder, Encoder *rightDriveEncoder): 
				RobotDrive(frontLeftMotor,rearLeftMotor,frontRightMotor,rearRightMotor), 
				shifter(gearShifter)
				,leftEncoder(leftDriveEncoder), rightEncoder(rightDriveEncoder),
				sampleIndex(0), shiftPoint(60.0), autoShiftEnabled(true)
{
	for(unsigned int i = 0; i < NUM_SPEED_SAMPLES; i++)
		speedHistory[i] = 0;
}
bool DriveTrain::isInHighGear() {
	return shifter->Get() == DoubleSolenoid::kForward;
}
void DriveTrain::toggleShiftGear() {
	shifter->Set(isInHighGear() ? DoubleSolenoid::kReverse : DoubleSolenoid::kForward);
}
void DriveTrain::shiftHighGear() {
	shifter->Set(DoubleSolenoid::kForward);
}
void DriveTrain::shiftLowGear() {
	shifter->Set(DoubleSolenoid::kReverse);
}
void DriveTrain::toogleAutoShiftEnable() {
	shiftPoint = BASELINE_SHIFT_POINT;
	autoShiftEnabled = !autoShiftEnabled;
}
void DriveTrain::setAutoShiftEnable(bool value) {
	shiftPoint = BASELINE_SHIFT_POINT;
	autoShiftEnabled = value;
}
bool DriveTrain::isAutoShiftEnabled() {
	return autoShiftEnabled;
}
void DriveTrain::takeSpeedSample() {
	double left = leftEncoder->GetRate();
	double right = rightEncoder->GetRate();
	speedHistory[sampleIndex++] = (fabs(left) > fabs(right)) ? left : right;	//should I use abolute values in the assignment
	if(sampleIndex >= NUM_SPEED_SAMPLES) sampleIndex = 0;
}
void DriveTrain::shiftAutomaically() {
	if(!autoShiftEnabled)return;
	//find average speed
	double avg = 0.0;
	for(unsigned int i = 0; i < NUM_SPEED_SAMPLES; i++)
		avg += speedHistory[i];
	avg /= ((double)NUM_SPEED_SAMPLES);
	if(fabs(avg) > shiftPoint) {
		shiftPoint = BASELINE_SHIFT_POINT - 8.0;
		shiftHighGear();
	} else/* if(fabs(avg) < shiftPoint - 5.0){*/{
		shiftPoint = BASELINE_SHIFT_POINT;
		shiftLowGear();		
	}
}
