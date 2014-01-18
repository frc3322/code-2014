#include "WPILib.h"
#include "../xbox.h"
#include "AutoShiftDrive.h"
#include "SmartDashboard/SmartDashboard.h"

class RobotDemo : public IterativeRobot
{
	Joystick stick;
	Talon leftFront, leftRear, rightFront, rightRear;
	Compressor compressor;
	DoubleSolenoid shifter;
	Encoder leftEncoder, rightEncoder;
	AutoShiftDrive drive;
public:
	RobotDemo():
		stick(1),
		leftFront(1), leftRear(2), rightFront(3), rightRear(4),
		compressor(5,1),
		shifter(1,2),	//forward ->lowGear ???
		leftEncoder(1,2,true, Encoder::k4X), rightEncoder(3,4,true, Encoder::k4X),
		drive(leftFront,leftRear,rightFront,rightRear,shifter)
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
	static bool currentY = false, previousY = false;
	drive.ArcadeDrive(stick,Joystick::kDefaultYAxis,stick,Joystick::kDefaultTwistAxis);
	//code for toggling high/low gear with ybutton
	currentY = stick.GetRawButton(YBUTTON);
	if(currentY && !previousY)
		drive.toggleShiftGear();
	previousY = currentY;
	SmartDashboard::PutNumber("left  encoder",leftEncoder.Get());
	SmartDashboard::PutNumber("right encoder",rightEncoder.Get());
}
void RobotDemo::TestInit() {
}
void RobotDemo::TestPeriodic() {
}
};
START_ROBOT_CLASS(RobotDemo);
