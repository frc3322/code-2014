#ifndef AUTO_SHIFT_DRIVE_H_
#define AUTO_SHIFT_DRIVE_H_
#include "wpilib.h"

class AutoShiftDrive : public RobotDrive {
private:
	DoubleSolenoid &shifter;
	//Encoder &leftEncoder, &rightEncoder;
public:
	AutoShiftDrive(SpeedController &frontLeftMotor, SpeedController &rearLeftMotor,
					SpeedController &frontRightMotor, SpeedController &rearRightMotor,
					DoubleSolenoid &gearShifter);
//					Encoder &leftDriveEncoder, Encoder &rightDriveEncoder);
	void toggleShiftGear();
	void shiftHighGear();
	void shiftHighLow();
	DoubleSolenoid::Value getShifterValue();
	void setShifterValue(DoubleSolenoid::Value val);
};

#endif
