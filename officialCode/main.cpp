#include "WPILib.h"
#include "../xbox.h"
#include "SmartDashboard/SmartDashboard.h"
#include "DriveTrain.h"

class Robot : public IterativeRobot
{
	Talon leftFront, leftRear, rightFront, rightRear;
	DoubleSolenoid gearShifter;
	DriveTrain drive;
	Joystick stick;
public:
	Robot():
		leftFront(1), leftRear(2), rightFront(3), rightRear(4),
		gearShifter(1,2),
		drive(&leftFront,&leftRear,&rightFront,&rightRear,&gearShifter,NULL,NULL),
		stick(1)
	{
		drive.SetExpiration(0.1);
		this->SetPeriod(0);
	}
void Robot::RobotInit() {
}
void Robot::DisabledInit() {
}
void Robot::DisabledPeriodic() {
}
void Robot::AutonomousInit() {
}
void Robot::AutonomousPeriodic() {
}
void Robot::TeleopInit() {
}
void Robot::TeleopPeriodic() {
	drive.ArcadeDrive(stick); // drive with arcade style 
}
void Robot::TestInit() {
}
void Robot::TestPeriodic() {
}
};
START_ROBOT_CLASS(Robot);
