#include "WPILib.h"
#include "../xbox.h"
#include "SmartDashboard/SmartDashboard.h"
#include "DriveTrain.h"
#include "Gatherer.h"
#include "TripleSpeedController.h"
#include "Shooter.h"

class Robot : public IterativeRobot
{
	Talon left1, left2, left3, right1, right2, right3;
	Talon arm, roller;
	DoubleSolenoid gearShifter;
	Compressor compressor;
	Encoder leftEncoder, rightEncoder;
	TripleSpeedController left, right;
	DriveTrain drive;
	Joystick stick;
	Joystick tech;
	AnalogChannel potentiometer;
	PIDController armController;
	Gatherer gatherer;
	double deadbandWidth;
public:
	Robot():
		left1(1), left2(2), left3(20), right1(3), right2(4), right3(20),
		arm(20), roller(20),
		gearShifter(1,2),
		compressor(5,1),
		leftEncoder(1,2,true, Encoder::k4X), rightEncoder(3,4,true, Encoder::k4X),
		left(&left1, &left2, &left3),right(&right1, &right2, &right3),
		drive(&left,&right,&gearShifter,&leftEncoder,&rightEncoder),
		stick(1),
		tech(2),
		potentiometer(2),
		armController(0.1,0.0,0.0,&potentiometer,&arm), //should get PID values from smartdashboard for the purpose of testing
		gatherer(&roller,&potentiometer,&armController),
		deadbandWidth(0.01)
	{
		drive.SetExpiration(0.1);
		this->SetPeriod(0);
	}
void Robot::RobotInit() {
	compressor.Start();
	leftEncoder.Start();
	rightEncoder.Start();
	SmartDashboard::init();
	//have to put numbers before you can get them
	SmartDashboard::PutNumber("baseline shift point", drive.baselineShiftPoint);
	SmartDashboard::PutNumber("shift point band width", drive.shiftbandWidth);
	SmartDashboard::PutNumber("joystick deadband", deadbandWidth);
	gatherer.init();
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
	float moveValue = stick.GetAxis(Joystick::kYAxis);
	float rotateValue = stick.GetAxis(Joystick::kTwistAxis);
	//deadband for joystick
	if(fabs(moveValue) < deadbandWidth)moveValue = 0.0;
	if(fabs(rotateValue) < deadbandWidth)rotateValue = 0.0;
	drive.ArcadeDrive(moveValue,rotateValue);
	//code for toggling high/low gear with ybutton
	currentY = stick.GetRawButton(YBUTTON);
	if(currentY && !previousY)
		drive.toggleShiftGear();
	previousY = currentY;
	SmartDashboard::PutNumber("left  encoder",leftEncoder.GetRate());
	SmartDashboard::PutNumber("right encoder",rightEncoder.GetRate());
	SmartDashboard::PutBoolean("Is auto shift enabled", drive.isAutoShiftEnabled());
	SmartDashboard::PutBoolean("Is in high gear", drive.isInHighGear());
	SmartDashboard::PutNumber("Arm angle", potentiometer.GetValue());
	drive.baselineShiftPoint = SmartDashboard::GetNumber("baseline shift point");
	drive.shiftbandWidth = SmartDashboard::GetNumber("shift point band width");
	deadbandWidth = fabs(SmartDashboard::GetNumber("joystick deadband"));
	SmartDashboard::PutNumber("y axis",stick.GetAxis(Joystick::kYAxis));
	SmartDashboard::PutNumber("t axis",stick.GetAxis(Joystick::kTwistAxis));
}
void Robot::TestInit() {
}
void Robot::TestPeriodic() {
}
};
START_ROBOT_CLASS(Robot);
