#include "DriveTrain.h"

DriveTrain::DriveTrain(SpeedController *frontLeftMotor, SpeedController *rearLeftMotor,
				SpeedController *frontRightMotor, SpeedController *rearRightMotor)://,
				//Encoder &leftDriveEncoder, Encoder &rightDriveEncoder): 
				RobotDrive(frontLeftMotor,rearLeftMotor,frontRightMotor,rearRightMotor)
				//shifter(gearShifter)
				//leftEncoder(leftDriveEncoder), rightEncoder(rightDriveEncoder)
{
}
/*
bool DriveTrain::isInForwardGear() {
	return shifter->Get() == DoubleSolenoid::kForward;
}
void DriveTrain::toggleShiftGear() {
	shifter->Set(isInForwardGear() ? DoubleSolenoid::kReverse : DoubleSolenoid::kForward);
}
void DriveTrain::shiftHighGear() {
	shifter->Set(DoubleSolenoid::kForward);
}
void DriveTrain::shiftHighLow() {
	shifter->Set(DoubleSolenoid::kReverse);
}
DoubleSolenoid::Value DriveTrain::getShifterValue() {
	return shifter->Get();
}
void DriveTrain::setShifterValue(DoubleSolenoid::Value val) {
	shifter->Set(val);
}
*/
