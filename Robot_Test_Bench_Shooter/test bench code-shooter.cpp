#include "WPILib.h"


/**
 * This is a demo program showing the use of the RobotBase class.
 * The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 */ 
class RobotDemo : public IterativeRobot
{
	Joystick stick; // only joystick
	Compressor pump;
	Solenoid sole1;
	Solenoid sole2;
	Talon talon1;
	Talon talon2;
	AnalogChannel pot;

public:
	RobotDemo():
		stick(1),		// as they are declared above.
		pump(1,1),
		sole1(1),
		sole2(2),
		talon1(1),
		talon2(2),
		pot(1)
	{
		this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
	}

void RobotDemo::RobotInit() {
	pump.Enabled();
	
}

void RobotDemo::DisabledInit() {
}

void RobotDemo::DisabledPeriodic() {
}

void RobotDemo::AutonomousInit() {
}

void RobotDemo::AutonomousPeriodic() {
	
}

void RobotDemo::TeleopInit() {
	//pump.Start();
}

void RobotDemo::TeleopPeriodic() {
	if(stick.GetRawButton(1)) {
		sole1.Set(1);
		sole2.Set(0);
		
	} else {
		sole1.Set(0);
		sole2.Set(1);
	}
	talon1.Set(stick.GetY());
	/*if(stick.GetRawButton(2)) {
		talon1.Set(1);
	} else {
		talon1.Set(0);
	}
	*?/
	if(pot.GetValue()>50) {
		if(stick.GetRawButton(3)) {
			talon2.Set(1);
		} else {
			talon2.Set(0);
		}
	} else {
		talon2.Set(0);
	}
	//myRobot.
	
	//ArcadeDrive(stick); // drive with arcade style */

}

void RobotDemo::TestInit() {
}

void RobotDemo::TestPeriodic() {
}

};

START_ROBOT_CLASS(RobotDemo);

