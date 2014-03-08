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
		const double FORWARD_POSITION;
		const double BACKWARD_POSITION;
		const double UP_POSITION;
		const double DOWN_POSITION;
};
#endif
