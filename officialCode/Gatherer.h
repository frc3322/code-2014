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
		double offset;
	public:
		Gatherer(Talon *roller, AnalogChannel *armAngle,
				PIDController *armController,
				double offset = 0.0);
		void rollerControl(double rollerSpeed);
		void setArmAngle(double value);
		bool isPIDEnabled();
		void setPIDEnabled(bool value);
		void togglePIDEnabled();
		void setDownPosition(double downPos);
		double FORWARD_POSITION;
		double BACKWARD_POSITION;
		double UP_POSITION;
		double DOWN_POSITION;
};
#endif
