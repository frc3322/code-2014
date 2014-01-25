#include "WPILib.h"
#include "../xbox.h"
#include "SmartDashboard/SmartDashboard.h"
#include "DriveTrain.h"

class Robot : public IterativeRobot
{
	Talon leftFront, leftRear, rightFront, rightRear;
	DoubleSolenoid gearShifter;
	Compressor compressor;
	Encoder leftEncoder, rightEncoder;
	DriveTrain drive;
	Joystick stick;
public:
	Robot():
		leftFront(1), leftRear(2), rightFront(3), rightRear(4),
		gearShifter(1,2),
		compressor(5,1),
		leftEncoder(1,2,true, Encoder::k4X), rightEncoder(3,4,true, Encoder::k4X),
		drive(&leftFront,&leftRear,&rightFront,&rightRear,&gearShifter,&leftEncoder,&rightEncoder),
		stick(1)
	{
		drive.SetExpiration(0.1);
		this->SetPeriod(0);
	}
void Robot::RobotInit() {
	compressor.Start();
	leftEncoder.Start();
	rightEncoder.Start();
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
	static bool currentY = false, previousY = false, currentB = false, previousB = false;
	if(stick.GetRawButton(ABUTTON)) {
		leftEncoder.Reset();
		rightEncoder.Reset();
	}
	//toggle auto shift enabled
	currentB = stick.GetRawButton(BBUTTON);
	if(currentB && !previousB)
		drive.toogleAutoShiftEnable();
	previousB = currentB;
	drive.takeSpeedSample();
	drive.shiftAutomaically();
	drive.ArcadeDrive(stick,Joystick::kDefaultYAxis,stick,Joystick::kDefaultTwistAxis);
	//code for toggling high/low gear with ybutton
	currentY = stick.GetRawButton(YBUTTON);
	if(currentY && !previousY)
		drive.toggleShiftGear();
	previousY = currentY;
	SmartDashboard::PutNumber("left  encoder",leftEncoder.GetRate());
	SmartDashboard::PutNumber("right encoder",rightEncoder.GetRate());
	SmartDashboard::PutBoolean("Is auto shift enabled", drive.isAutoShiftEnabled());
	SmartDashboard::PutBoolean("Is in high gear", drive.isInHighGear());
}
void Robot::TestInit() {
}
void Robot::TestPeriodic() {
}
};
START_ROBOT_CLASS(Robot);
