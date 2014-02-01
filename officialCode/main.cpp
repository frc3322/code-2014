#include "WPILib.h"
#include "../xbox.h"
#include "SmartDashboard/SmartDashboard.h"
#include "DriveTrain.h"
#include "Gatherer.h"
#include "GathererArm.h"
#include "TripleSpeedController.h"
#include "Shooter.h"

class Robot : public IterativeRobot
{
	Talon left1, left2, left3, right1, right2, right3;
	Talon leftArm, rightArm, leftRoller, rightRoller;
	DoubleSolenoid gearShifter;
	Compressor compressor;
	Encoder leftEncoder, rightEncoder;
	TripleSpeedController left, right;
	DriveTrain drive;
	Joystick stick;
	Joystick tech;
	AnalogChannel potentiometer;
	GathererArm arm;
	PIDController armController;
	Gatherer gatherer;
public:
	Robot():
		left1(1), left2(2), left3(20), right1(3), right2(4), right3(20),
		leftArm(20), rightArm(20), leftRoller(20), rightRoller(20),
		gearShifter(1,2),
		compressor(5,1),
		leftEncoder(1,2,true, Encoder::k4X), rightEncoder(3,4,true, Encoder::k4X),
		left(&left1, &left2, &left3),right(&right1, &right2, &right3),
		drive(&left,&right,&gearShifter,&leftEncoder,&rightEncoder),
		stick(1),
		tech(2),
		potentiometer(2),
		arm(&leftArm,&rightArm),
		armController(0.1,0.001,0.0,&potentiometer,&arm), //should get PID values from smartdashboard for the purpose of testing
		gatherer(&leftRoller,&rightRoller,&potentiometer,&armController)
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
	SmartDashboard::PutNumber("Arm angle", potentiometer.GetValue());
	drive.baselineShiftPoint = SmartDashboard::GetNumber("baseline shift point");
	drive.shiftbandWidth = SmartDashboard::GetNumber("shift point band width");
}
void Robot::TestInit() {
}
void Robot::TestPeriodic() {
}
};
START_ROBOT_CLASS(Robot);
