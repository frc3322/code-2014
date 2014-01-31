#include "wpilib.h"

class Gatherer {
	private:
		Talon *leftRoller, *rightRoller;
		AnalogChannel *armAngle;
		PIDController *armController;
	public:
		Gatherer(Talon *rollerLeft, Talon *rollerRight, AnalogChannel *armAngle,
				PIDController *armController);
		void rollerControl(double rollerSpeed);
		void setArmAngle(double value);
		void moveArmForward(double increment);
		void moveArmBackward(double increment);	//redundent?? could just pass negatives to moveArm
};
