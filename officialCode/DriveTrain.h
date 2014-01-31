#ifndef AUTO_SHIFT_DRIVE_H_
#define AUTO_SHIFT_DRIVE_H_
#include "wpilib.h"
#include "math.h"

const unsigned int NUM_SPEED_SAMPLES = 4;
//const double BASELINE_SHIFT_POINT = 1500.0;
class DriveTrain : public RobotDrive {
private:
	DoubleSolenoid *shifter;
	Encoder *leftEncoder, *rightEncoder;
	double speedHistory[NUM_SPEED_SAMPLES];
	unsigned int sampleIndex;
	double shiftPoint;
	bool autoShiftEnabled;
	//RobotDrive
public:
	double baselineShiftPoint;
	double shiftbandWidth;
	DriveTrain(SpeedController *frontLeftMotor, SpeedController *rearLeftMotor,
					SpeedController *frontRightMotor, SpeedController *rearRightMotor,
					DoubleSolenoid *gearShiter,
					Encoder *leftDriveEncoder, Encoder *rightDriveEncoder);
	bool isInHighGear();
	void toggleShiftGear();
	void shiftHighGear();
	void shiftLowGear();
	void toogleAutoShiftEnable();
	void setAutoShiftEnable(bool value);
	bool isAutoShiftEnabled();
	void takeSpeedSample();
	void shiftAutomaically();
	bool isTurning();
};

#endif
