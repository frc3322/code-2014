#include "DriveTrain.h"

DriveTrain::DriveTrain(SpeedController *frontLeftMotor, SpeedController *rearLeftMotor,
				SpeedController *frontRightMotor, SpeedController *rearRightMotor,
				DoubleSolenoid *gearShifter,
				Encoder *leftDriveEncoder, Encoder *rightDriveEncoder): 
				RobotDrive(frontLeftMotor,rearLeftMotor,frontRightMotor,rearRightMotor), 
				shifter(gearShifter)
				,leftEncoder(leftDriveEncoder), rightEncoder(rightDriveEncoder)
{
}
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
