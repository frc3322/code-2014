#ifndef AUTO_SHIFT_DRIVE_H_
#define AUTO_SHIFT_DRIVE_H_
#include "wpilib.h"
#include "math.h"

const unsigned int NUM_SPEED_SAMPLES = 25;  //at 50 samples per second, this sample size takes half a second.
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
	//double baselineShiftPoint;
	//double shiftbandWidth;
	double lowHighShiftPoint;
	double highLowShiftPoint;
	double timeShiftHigh;
	double timeShiftLow;
	DriveTrain(SpeedController *frontLeftMotor, SpeedController *rearLeftMotor,
			SpeedController *frontRightMotor, SpeedController *rearRightMotor,
			DoubleSolenoid *gearShiter,
			Encoder *leftDriveEncoder, Encoder *rightDriveEncoder);
	DriveTrain(SpeedController *left, SpeedController *right,
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
