#include "WPILib.h"
#include "../xbox.h"

class RobotDemo : public IterativeRobot
{
	Joystick stick;
	Talon leftFront, leftRear, rightFront, rightRear;
	RobotDrive drive;
	Compressor compressor;
	DoubleSolenoid shifter;
	bool highGear;
public:
	RobotDemo():
		stick(1),
		leftFront(1), leftRear(2), rightFront(3), rightRear(4),
		drive(leftFront,leftRear,rightFront,rightRear),
		compressor(5,1),
		shifter(1,2),	//forward ->lowGear ???
		highGear(false)
	{
		drive.SetExpiration(0.1);
		this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
	}
void RobotDemo::RobotInit() {
	compressor.Start();
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
}
void RobotDemo::TeleopPeriodic() {
	drive.ArcadeDrive(stick,Joystick::kDefaultYAxis,stick,Joystick::kDefaultTwistAxis);
	if(stick.GetRawButton(YBUTTON)) {	//code for toggling high/low gear with ybutton
		if(highGear)
			shifter.Set(DoubleSolenoid::kForward);
		else
			shifter.Set(DoubleSolenoid::kReverse);
		highGear = !highGear;
	}
}
void RobotDemo::TestInit() {
}
void RobotDemo::TestPeriodic() {
}
};
START_ROBOT_CLASS(RobotDemo);
