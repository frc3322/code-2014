#ifndef GATHERER_H
#define GATHERER_H
#include "wpilib.h"
#include "constants.h"

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
		bool isPIDEnabled();
		void setPIDEnabled(bool value);
		void togglePIDEnabled();
};
#endif
