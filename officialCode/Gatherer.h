#ifndef GATHERER_H
#define GATHERER_H
#include "wpilib.h"

class Gatherer {
	private:
		Talon *roller;
		AnalogChannel *armAngle;
		PIDController *armController;
		bool pidEnabled;
	public:
		Gatherer(Talon *roller, AnalogChannel *armAngle,
				PIDController *armController);
		void rollerControl(double rollerSpeed);
		void setArmAngle(double value);
		void moveArmForward(double increment);
		void moveArmBackward(double increment);	//redundent?? could just pass negatives to moveArm
		void init();
		bool isPIDEnabled();
		void setPIDEnabled(bool value);
		void togglePIDEnabled();
};
#endif
