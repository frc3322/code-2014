#include "WPILib.h"

class Robot : public IterativeRobot
{
	RobotDrive drive;
	Joystick stick;

public:
	Robot():
		drive(1, 2),
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
