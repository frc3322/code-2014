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
	Talon winch;
	Talon arm, roller;
	DoubleSolenoid gearShifter, trigger;
	Compressor compressor;
	Encoder leftEncoder, rightEncoder;
	TripleSpeedController left, right;
	DriveTrain drive;
	Joystick stick;
	Joystick tech;
	AnalogChannel potentiometer;
	AnalogChannel shooterPot;
	PIDController armController;
	Gatherer gatherer;
	Shooter shooter;
	double deadbandWidth;
	double Gather_angle;
	double P, I, D;
public:
	Robot():
		left1(1), left2(2), left3(3), right1(4), right2(5), right3(6),
		winch(7),
		arm(9), roller(8),
		gearShifter(1,2),
		trigger(3,4),
		compressor(5,1),
		leftEncoder(1,2,true, Encoder::k4X), rightEncoder(3,4,false, Encoder::k4X),
		left(&left1, &left2, &left3),right(&right1, &right2, &right3),
		drive(&left,&right,&gearShifter,&leftEncoder,&rightEncoder),
		stick(1),
		tech(2),
		potentiometer(2),
		shooterPot(3),
		armController(-1.0,0.0,0.0,&potentiometer,&arm), //should get PID values from smartdashboard for the purpose of testing
		gatherer(&roller,&potentiometer,&armController),
		shooter(&winch, &shooterPot, &trigger),
		deadbandWidth(0.01),P(-0.4), I(-0.01), D(0.0)
		
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
	SmartDashboard::PutNumber("joystick deadband", deadbandWidth);
	SmartDashboard::PutNumber("shift high point",drive.shiftHighPoint);
	SmartDashboard::PutNumber("shift low point",drive.shiftLowPoint);
	SmartDashboard::PutNumber("shift counter threshold",drive.shiftCounterThreshold);
	drive.shiftLowPoint = fabs(SmartDashboard::GetNumber("shift low point"));
	SmartDashboard::PutNumber("P",P);
	SmartDashboard::PutNumber("I",I);
	SmartDashboard::PutNumber("D",D);
	gatherer.init();
}
void Robot::PrintInfoToSmartDashboard() {
	SmartDashboard::PutNumber("left  encoder",leftEncoder.GetDistance());
	SmartDashboard::PutNumber("right encoder",rightEncoder.GetRate());
	SmartDashboard::PutBoolean("Is auto shift enabled", drive.isAutoShiftEnabled());
	SmartDashboard::PutBoolean("Is in high gear", drive.isInHighGear());
	SmartDashboard::PutNumber("Arm potentiometer", potentiometer.GetVoltage());
	deadbandWidth = fabs(SmartDashboard::GetNumber("joystick deadband"));
	drive.shiftHighPoint = fabs(SmartDashboard::GetNumber("shift high point"));
	drive.shiftLowPoint = fabs(SmartDashboard::GetNumber("shift low point"));
	drive.shiftCounterThreshold = (unsigned int)fabs(SmartDashboard::GetNumber("shift counter threshold"));
	SmartDashboard::PutNumber("y axis",stick.GetAxis(Joystick::kYAxis));
	SmartDashboard::PutNumber("t axis",stick.GetAxis(Joystick::kTwistAxis));
	SmartDashboard::PutBoolean("pid gather control enabled",gatherer.isPIDEnabled());
	P = SmartDashboard::GetNumber("P");
	I = SmartDashboard::GetNumber("I");
	D = SmartDashboard::GetNumber("D");
	armController.SetPID(P,I,D);
}
void Robot::DisabledInit() {
	gatherer.setPIDEnabled(false);
}
void Robot::DisabledPeriodic() {
	PrintInfoToSmartDashboard();
}
void Robot::AutonomousInit() {
	leftEncoder.SetDistancePerPulse(1.0/1000.0);
	rightEncoder.SetDistancePerPulse(1.0/1000.0);
	leftEncoder.Reset();
	rightEncoder.Reset();
	drive.shiftLowGear();
}
void Robot::AutonomousPeriodic() {
	PrintInfoToSmartDashboard();
	if(leftEncoder.GetDistance() < 15.2 && rightEncoder.GetDistance() < 15.2)
		drive.TankDrive(-0.7, -0.705);
	else drive.TankDrive(0.0,0.0);
}
void Robot::TeleopInit() {
	leftEncoder.SetDistancePerPulse(1.0);
	rightEncoder.SetDistancePerPulse(1.0);
}
void Robot::TeleopPeriodic() {
	static bool currentY = false, previousY = false, currentB = false, previousB = false;
	static bool currentX = false, previousX = false;
	
	if(stick.GetRawButton(ABUTTON)) {
		leftEncoder.Reset();
		rightEncoder.Reset();
	}
	//toggle auto shift enabled
	currentB = stick.GetRawButton(BBUTTON);
	if(currentB && !previousB){
		//drive.toogleAutoShiftEnable();
		shooter.retractTrigger();
	}
	previousB = currentB;
	drive.takeSpeedSample();
	drive.shiftAutomatically();
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
	if(stick.GetRawButton(RBUMPER))
		gatherer.rollerControl(1);
	else
		gatherer.rollerControl(0);
	currentX = stick.GetRawButton(XBUTTON);
	if(currentX && !previousX)
		shooter.fireTrigger();
//		gatherer.togglePIDEnabled();
	//gatherer.moveArmForward(stick.GetAxis(Joystick::kThrottleAxis)*0.1);
	if(gatherer.isPIDEnabled()) {
		if(stick.GetRawButton(ABUTTON))
			gatherer.setArmAngle(3.7);
		else
			gatherer.setArmAngle(4.3);
	} else {
		arm.Set(stick.GetAxis(Joystick::kThrottleAxis)*0.5);
	}
	previousX = currentX;
	previousY = currentY;
	PrintInfoToSmartDashboard();

	if(stick.GetRawButton(LBUMPER))
		shooter.runWinch();
	else
		shooter.stopWinch();

	static bool currentBack = false, previousBack = false;

	//bool isRetracted = true;
//	
	currentBack = stick.GetRawButton(BACK);
	if(currentBack && !previousBack) {
		shooter.fireTrigger();
	}
	previousBack = currentBack;	
}
void Robot::TestInit() {
}
void Robot::TestPeriodic() {
}
};
START_ROBOT_CLASS(Robot);
