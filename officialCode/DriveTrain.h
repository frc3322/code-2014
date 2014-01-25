#ifndef AUTO_SHIFT_DRIVE_H_
#define AUTO_SHIFT_DRIVE_H_
#include "wpilib.h"

class DriveTrain : public RobotDrive {
private:
	DoubleSolenoid *shifter;
	//Encoder &leftEncoder, &rightEncoder;
public:
	DriveTrain(SpeedController *frontLeftMotor, SpeedController *rearLeftMotor,
					SpeedController *frontRightMotor, SpeedController *rearRightMotor);
//					Encoder &leftDriveEncoder, Encoder &rightDriveEncoder);
	bool isInForwardGear();
	void toggleShiftGear();
	void shiftHighGear();
	void shiftHighLow();
	DoubleSolenoid::Value getShifterValue();
	void setShifterValue(DoubleSolenoid::Value val);
	
};

#endif
