#include "WPILib.h"
#include "../xbox.h"
#include "SmartDashboard/SmartDashboard.h"
#include "DriveTrain.h"
#include "Gatherer.h"
#include "TripleSpeedController.h"
#include "Shooter.h"
#include "constants.h"

double ceiling(double a, double c) {
	return a > c ? c : a;
}
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
	Joystick stick;	//both are xbox controllers
	Joystick tech;
	AnalogChannel potentiometer;
	AnalogChannel shooterPot;
	PIDController armController;
	Gatherer gatherer;
	Shooter shooter;
	double deadbandWidth;
	double Gather_angle;
	double P, I, D;
	unsigned int autonMode;
	double autonStartTime;
public:
	Robot():
		left1(1), left2(2), left3(3), right1(4), right2(5), right3(6),
		winch(7),
		arm(9), roller(8),
		gearShifter(1,2),
#if ROBOT == COMP
		trigger(3,4),
#else
		trigger(4,3),
#endif
		compressor(5,1),
#if ROBOT == COMP
		leftEncoder(1,2,true, Encoder::k4X), rightEncoder(3,4,false, Encoder::k4X),
#else
		leftEncoder(1,2,false, Encoder::k4X), rightEncoder(3,4,false, Encoder::k4X),
#endif
		left(&left1, &left2, &left3),right(&right1, &right2, &right3),
		drive(&left,&right,&gearShifter,&leftEncoder,&rightEncoder),
		stick(1),
		tech(2),
		potentiometer(2),
		shooterPot(1),
		armController(-1.0,0.0,0.0,&potentiometer,&arm), //should get PID values from smartdashboard for the purpose of testing
		gatherer(&roller,&potentiometer,&armController),
		shooter(&winch, &shooterPot, &trigger),
		deadbandWidth(0.01),
		P(-0.4), I(-0.01), D(0.0),
		autonMode(0)
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
	SmartDashboard::PutBoolean("Is auto shift enabled", drive.isAutoShiftEnabled());
	SmartDashboard::PutNumber("shift high point",drive.shiftHighPoint);
	SmartDashboard::PutNumber("shift low point",drive.shiftLowPoint);
	SmartDashboard::PutNumber("shift counter threshold",drive.shiftCounterThreshold);
	drive.shiftLowPoint = fabs(SmartDashboard::GetNumber("shift low point"));
	SmartDashboard::PutNumber("P",P);
	SmartDashboard::PutNumber("I",I);
	SmartDashboard::PutNumber("D",D);
	SmartDashboard::PutNumber("shooter low threshold",shooter.lowThreshold);
	SmartDashboard::PutNumber("shooter high threshold",shooter.highThreshold);
	SmartDashboard::PutNumber("auton mode", autonMode);
	SmartDashboard::PutBoolean("pid gather control enabled",gatherer.isPIDEnabled());
	char label[3];
	label[0] = '0';
	label[1] = ')';
	label[2] = '\0';
	const char* descriptions[] = {
			"Drive to low goal",
			"Drive to low goal and reverse gatherer to feed ball into low goal",
			"Drive, pull down shooter, fire"
	};
	for(unsigned int i = 0; i < sizeof(descriptions)/sizeof(const char*); i++){
		SmartDashboard::PutString(label,descriptions[i]);
		label[0] = label[0] < '9' ? label[0] + 1 : '0';
	}
}
void Robot::PrintInfoToSmartDashboard() {
	static bool pidEnabled = SmartDashboard::GetBoolean("pid gather control enabled");
	SmartDashboard::PutNumber("left  encoder",leftEncoder.GetDistance());
	SmartDashboard::PutNumber("right encoder",rightEncoder.GetDistance());	
	drive.setAutoShiftEnable(SmartDashboard::GetBoolean("Is auto shift enabled"));
	SmartDashboard::PutBoolean("Is in high gear", drive.isInHighGear());
	SmartDashboard::PutNumber("Arm potentiometer", potentiometer.GetVoltage());
	deadbandWidth = fabs(SmartDashboard::GetNumber("joystick deadband"));
	drive.shiftHighPoint = fabs(SmartDashboard::GetNumber("shift high point"));
	drive.shiftLowPoint = fabs(SmartDashboard::GetNumber("shift low point"));
	drive.shiftCounterThreshold = (unsigned int)fabs(SmartDashboard::GetNumber("shift counter threshold"));	//if(pidEnabled)
	if(SmartDashboard::GetBoolean("pid gather control enabled") != pidEnabled){
		gatherer.setPIDEnabled(!pidEnabled);
		pidEnabled = !pidEnabled;
	}
	P = SmartDashboard::GetNumber("P");
	I = SmartDashboard::GetNumber("I");
	D = SmartDashboard::GetNumber("D");
	armController.SetPID(P,I,D);
	SmartDashboard::PutNumber("shooter pot", shooterPot.GetVoltage());
	shooter.highThreshold = ceiling(fabs(SmartDashboard::GetNumber("shooter high threshold")),5.0);
	shooter.lowThreshold = ceiling(fabs(SmartDashboard::GetNumber("shooter low threshold")),5.0);
	SmartDashboard::PutBoolean("shooter ready",shooter.isReadyToShoot());
	autonMode = abs((int)SmartDashboard::GetNumber("auton mode"));
	SmartDashboard::PutNumber("FPGA time", Timer::GetFPGATimestamp());
	SmartDashboard::PutNumber("PPC time", Timer::GetFPGATimestamp());
}
void Robot::DisabledInit() {
	gatherer.setPIDEnabled(false);
}
void Robot::DisabledPeriodic() {
	PrintInfoToSmartDashboard();
}
void Robot::AutonomousInit() {
	leftEncoder.SetDistancePerPulse(1.0/1000.0);	//should tweek these values
	rightEncoder.SetDistancePerPulse(1.0/1000.0);
	leftEncoder.Reset();
	rightEncoder.Reset();
	drive.shiftLowGear();
	autonStartTime = Timer::GetPPCTimestamp();
}
bool Robot::driveForward(double distance, double speed, double timeout) {
	double leftDistance = leftEncoder.GetDistance();
	double rightDistance = rightEncoder.GetDistance();
	if(leftDistance < distance && rightDistance < distance && Timer::GetPPCTimestamp() < timeout) {
		if(leftDistance + 1 > distance || rightDistance + 1.0 > distance)	//if within a foot
			speed *= 0.5;
		drive.ArcadeDrive(-speed,-(leftDistance - rightDistance)*5.5);
		return false;
	}
	drive.ArcadeDrive(0.0,0.0);
	return true;
}
void Robot::AutonomousPeriodic() {
	PrintInfoToSmartDashboard();
	static const bool hasReachedDestination = true;
	switch (autonMode) {
	case 0:	//Drive to low goal
		driveForward(15.2, 0.7, autonStartTime + 7.0);
	break;
	case 1:	//Drive to low goal and reverse gatherer to feed ball into low goal
		gatherer.setArmAngle(1.9);		//pull back gatherer arm
		if(driveForward(15.2, 0.7, autonStartTime + 7.0) == hasReachedDestination)//drive forward to low goal
			gatherer.rollerControl(-1);		//run rollers to put ball in low goal
	break;
	case 2:
		if(driveForward(10.0, 0.7, autonStartTime + 10.0) == hasReachedDestination) {
			if(Timer::GetPPCTimestamp() > (autonStartTime + 9.5)) {
				shooter.releaseWinch();
			}
		} else
			shooter.engageWinch();
		if(Timer::GetPPCTimestamp() < (autonStartTime +4.2)) {
			shooter.runWinch();
		} else
			shooter.stopWinch();
			
		break;
	}
}
void Robot::TeleopInit() {
	leftEncoder.SetDistancePerPulse(1.0);
	rightEncoder.SetDistancePerPulse(1.0);
}
void Robot::TeleopPeriodic() {
	drive.takeSpeedSample();
	drive.shiftAutomatically();
	float moveValue = stick.GetAxis(Joystick::kYAxis);
	float rotateValue = stick.GetAxis(Joystick::kTwistAxis);
	//deadband for joystick
	if(fabs(moveValue) < deadbandWidth)moveValue = 0.0;
	if(fabs(rotateValue) < deadbandWidth)rotateValue = 0.0;
	//drive code
	drive.ArcadeDrive(moveValue,rotateValue);
	//run roller
	if(stick.GetRawButton(RBUMPER)||tech.GetRawButton(RBUMPER))
		gatherer.rollerControl(1);
	else if (stick.GetRawButton(LBUMPER)||tech.GetRawButton(LBUMPER)) //run roller backward
		gatherer.rollerControl(-1);
	else
		gatherer.rollerControl(0);
	//catapault
	if(stick.GetRawAxis(Joystick::kTwistAxis) > 0.8) { //left trigger
		shooter.stopWinch();
		shooter.releaseWinch();
	}else if(stick.GetRawAxis(Joystick::kTwistAxis) < -0.8) {//right trigger
		if(!shooter.isWinchEngaged())
			shooter.engageWinch();
		shooter.runWinch();
	}else {
		shooter.stopWinch();
	}
	arm.Set(tech.GetAxis(Joystick::kThrottleAxis) * 0.5); //Manual Gatherer control
	/*static bool currentB = false, previousB = false;
	static bool currentX = false, previousX = false;
	static bool currentY = false, previousY = false;
	static bool currentBack = false, previousBack = false;
	static bool currentStart = false, previousStart = false;
	static bool manualShooterControl = true;
	currentB = stick.GetRawButton(BBUTTON);
	currentX = stick.GetRawButton(XBUTTON);
	currentY = stick.GetRawButton(YBUTTON);
	currentBack = stick.GetRawButton(BACK);
	currentStart = stick.GetRawButton(START);
	//reset encoder
	if(stick.GetRawButton(ABUTTON)) {
		leftEncoder.Reset();
		rightEncoder.Reset();
	}*/	/*//code for toggling high/low gear with ybutton
	if(currentY && !previousY)
		drive.toggleShiftGear();*/
	/*
	if(gatherer.isPIDEnabled()) { //PID gatherer control
		if(stick.GetRawButton(ABUTTON)) {
			gatherer.setArmAngle(2.65);
		} else {
			gatherer.setArmAngle(3);
		}
	} else {
		
	}*/
	///toggle manual shooter control
	//if(currentStart && !previousStart)manualShooterControl = !manualShooterControl;
	/*if(manualShooterControl) {
		//release winch (shoot)
		if(currentX)
			shooter.engageWinch();
		else if(currentB)			//engage winch
			shooter.releaseWinch();
		//run winch
		if(stick.GetRawButton(LBUMPER))
			shooter.runWinch();
		else
			shooter.stopWinch();
	} else {
		shooter.runShooter(stick.GetRawButton(LBUMPER));
	}
	previousStart = currentStart;
	previousBack = currentBack;	
	previousX = currentX;
	previousY = currentY;
	previousB = currentB;*/
	PrintInfoToSmartDashboard();
}
void Robot::TestInit() {
}
void Robot::TestPeriodic() {
}
};
START_ROBOT_CLASS(Robot);
