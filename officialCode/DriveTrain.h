#ifndef AUTO_SHIFT_DRIVE_H_
#define AUTO_SHIFT_DRIVE_H_
#include "wpilib.h"
#include "math.h"
#include "constants.h"

const unsigned int NUM_SPEED_SAMPLES = 25;
class DriveTrain : public RobotDrive {
private:
	DoubleSolenoid *shifter;
	Encoder *leftEncoder, *rightEncoder;
	double leftSpeedHistory[NUM_SPEED_SAMPLES];
	double rightSpeedHistory[NUM_SPEED_SAMPLES];
	unsigned int sampleIndex;
	double shiftPoint;
	bool autoShiftEnabled;
public:
	double shiftHighPoint;
	double shiftLowPoint;
	unsigned int shiftHighCounter;
	unsigned int shiftLowCounter;
	unsigned int shiftCounterThreshold;
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
	void shiftAutomatically();
	bool isTurning();
};
#endif
