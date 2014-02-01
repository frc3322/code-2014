#include "DriveTrain.h"

DriveTrain::DriveTrain(SpeedController *frontLeftMotor, SpeedController *rearLeftMotor,
				SpeedController *frontRightMotor, SpeedController *rearRightMotor,
				DoubleSolenoid *gearShifter,
				Encoder *leftDriveEncoder, Encoder *rightDriveEncoder): 
				RobotDrive(frontLeftMotor,rearLeftMotor,frontRightMotor,rearRightMotor), 
				shifter(gearShifter)
				,leftEncoder(leftDriveEncoder), rightEncoder(rightDriveEncoder),
				sampleIndex(0), shiftPoint(60.0), autoShiftEnabled(true),
				baselineShiftPoint(1500.0),shiftbandWidth(15.0)
{
	for(unsigned int i = 0; i < NUM_SPEED_SAMPLES; i++)
		speedHistory[i] = 0;
}
DriveTrain::DriveTrain(SpeedController *left, SpeedController *right,
				DoubleSolenoid *gearShifter,
				Encoder *leftDriveEncoder, Encoder *rightDriveEncoder): 
				RobotDrive(left,right), 
				shifter(gearShifter),
				leftEncoder(leftDriveEncoder), rightEncoder(rightDriveEncoder),
				sampleIndex(0), shiftPoint(60.0), autoShiftEnabled(true),
				baselineShiftPoint(1500.0),shiftbandWidth(15.0)
{
	for(unsigned int i = 0; i < NUM_SPEED_SAMPLES; i++)
		speedHistory[i] = 0;
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
	shiftPoint = baselineShiftPoint;
	autoShiftEnabled = !autoShiftEnabled;
}
void DriveTrain::setAutoShiftEnable(bool value) {
	shiftPoint = baselineShiftPoint;
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
	if(isTurning()) {
		//do nothing
	} else if(fabs(avg) > shiftPoint + shiftbandWidth) {
		shiftPoint = baselineShiftPoint;// - 8.0;
		shiftHighGear();
	} else if(fabs(avg) < shiftPoint - shiftbandWidth){
		shiftPoint = baselineShiftPoint;
		shiftLowGear();		
	}
}
bool DriveTrain::isTurning() {
	//if encoders have opposite signs and
	return leftEncoder->GetRate() * rightEncoder->GetRate() < 0 ;
}
