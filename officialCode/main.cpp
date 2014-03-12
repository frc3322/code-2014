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
double floor(double a, double f) {
	return a < f ? f : a;
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
	AnalogChannel gathererPot;
	AnalogChannel shooterPot;
	PIDController armController;
	Gatherer gatherer;
	Shooter shooter;
	double deadbandWidth;
	double Gather_angle;
	double P, I, D;
	unsigned int autonMode;
	double autonStartTime;
	double autonDistance;
	double autonSpeed;
	double autonDriveTimeout;
	double autonShooterPosition;
	double autonShotDelay;
	double timeOfLastShot;
	double autonWinchTimeout;
public:
	Robot():
		left1(1), left2(2), left3(3), right1(4), right2(5), right3(6),
		winch(7), arm(9), roller(8), gearShifter(1,2),
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
		stick(1),tech(2),
		gathererPot(2), shooterPot(1),
		armController(-1.0,0.0,0.0,&gathererPot,&arm), //should get PID values from smartdashboard for the purpose of testing
		gatherer(&roller,&gathererPot,&armController),
		shooter(&winch, &shooterPot, &trigger),
		deadbandWidth(0.01),
#if ROBOT==COMP
		P(.2), I(0.0), D(0.0),
#else
		P(1.0), I(0.01), D(0.5),
#endif
		autonMode(0), autonStartTime(0.0), autonDistance(15.2), autonSpeed(0.7), autonDriveTimeout(10.0),
		autonShooterPosition(0.9), autonShotDelay(1.0), autonWinchTimeout(5.0)
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
	SmartDashboard::PutNumber("autonStartTime",autonStartTime);
	SmartDashboard::PutNumber("autonDistance",autonDistance);
	SmartDashboard::PutNumber("autonSpeed",autonSpeed);
	SmartDashboard::PutNumber("autonDriveTimeout",autonDriveTimeout);
	SmartDashboard::PutNumber("autonShooterPosition",autonShooterPosition);
	SmartDashboard::PutNumber("autonShotDelay",autonShotDelay);
	SmartDashboard::PutNumber("autonWinchTimeout",autonWinchTimeout);
}
void Robot::PrintInfoToSmartDashboard() {
	SmartDashboard::PutNumber("left  encoder",leftEncoder.GetDistance());
	SmartDashboard::PutNumber("right encoder",rightEncoder.GetDistance());	
	//drive.setAutoShiftEnable(SmartDashboard::GetBoolean("Is auto shift enabled"));
	SmartDashboard::PutBoolean("auto shift enabled", drive.isAutoShiftEnabled());
	SmartDashboard::PutNumber("Shooter potentiometer", shooterPot.GetVoltage());
	SmartDashboard::PutBoolean("Is in high gear", drive.isInHighGear());
	SmartDashboard::PutNumber("Arm potentiometer", gathererPot.GetVoltage());
	deadbandWidth = fabs(SmartDashboard::GetNumber("joystick deadband"));
	drive.shiftHighPoint = fabs(SmartDashboard::GetNumber("shift high point"));
	drive.shiftLowPoint = fabs(SmartDashboard::GetNumber("shift low point"));
	drive.shiftCounterThreshold = (unsigned int)fabs(SmartDashboard::GetNumber("shift counter threshold"));	//if(pidEnabled)
	P = SmartDashboard::GetNumber("P");
	I = SmartDashboard::GetNumber("I");
	D = SmartDashboard::GetNumber("D");
	armController.SetPID(P,I,D);
	SmartDashboard::PutNumber("shooter pot", shooterPot.GetVoltage());
	shooter.highThreshold = ceiling(fabs(SmartDashboard::GetNumber("shooter high threshold")),5.0);
	shooter.lowThreshold = ceiling(fabs(SmartDashboard::GetNumber("shooter low threshold")),5.0);
	SmartDashboard::PutBoolean("shooter ready",shooter.isReadyToShoot());
	autonMode = abs((int)SmartDashboard::GetNumber("auton mode"));
	SmartDashboard::PutNumber("PPC time", Timer::GetPPCTimestamp());
	SmartDashboard::PutBoolean("auto load catapult",shooter.autoLoad);
	SmartDashboard::PutNumber("tech trigger axis",tech.GetRawAxis(Joystick::kTwistAxis));
	SmartDashboard::PutNumber("autonStartTime",autonStartTime);
	autonDistance = SmartDashboard::GetNumber("autonDistance");
	autonSpeed = SmartDashboard::GetNumber("autonSpeed");
	autonDriveTimeout = fabs(SmartDashboard::GetNumber("autonDriveTimeout"));
//	autonShooterPosition = floor(fabs(SmartDashboard::GetNumber("autonShooterPosition")),shooter.POT_MIN);	//don't set it too low //I don't think floor means what you think it means
	autonShooterPosition = SmartDashboard::GetNumber("autonShooterPosition");	
	SmartDashboard::PutNumber("autonShooterPosition",autonShooterPosition);
	autonShotDelay = fabs(SmartDashboard::GetNumber("autonShotDelay"));
	autonWinchTimeout = fabs(SmartDashboard::GetNumber("autonWinchTimeout"));

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
	autonShooterPosition = shooter.POT_MIN;
	shooter.engageWinch();
	gatherer.setPIDEnabled(false);
	
}
bool Robot::driveForward(double distance, double speed, double timeout) {
	double leftDistance = leftEncoder.GetDistance();
	double rightDistance = rightEncoder.GetDistance();
	if(leftDistance < distance && rightDistance < distance && Timer::GetPPCTimestamp() < timeout) {
		if(leftDistance + 1.0 > distance || rightDistance + 1.0 > distance)	//if within a foot
			speed = speed * 0.5;
		drive.ArcadeDrive(-speed,-(leftDistance - rightDistance)*5.5);
		return false;
	}
	drive.ArcadeDrive(0.0,0.0);
	return true;
}
void Robot::AutonomousPeriodic() {
	PrintInfoToSmartDashboard();
	static const bool hasReachedDestination = true;
	static bool hasShot = false;
	static bool hasGathered = false;
	switch (autonMode) {
	case 0:	//Drive to low goal
		driveForward(autonDistance, autonSpeed, autonStartTime + autonDriveTimeout);
	break;
	case 1:	//Drive to low goal and reverse gatherer to feed ball into low goal
		gatherer.setArmAngle(gatherer.BACKWARD_POSITION);		//pull back gatherer arm
		if(driveForward(autonDistance, autonSpeed, autonStartTime + autonDriveTimeout) == hasReachedDestination)//drive forward to low goal
			gatherer.rollerControl(-1);		//run rollers to put ball in low goal
	break;
	case 2:
		if(driveForward(autonDistance, autonSpeed, autonStartTime + autonDriveTimeout) == hasReachedDestination)
			if(shooterPot.GetVoltage() < autonShooterPosition && shooter.isWinchEngaged()) {
				shooter.releaseWinch();
			}
		if(shooterPot.GetVoltage() > autonShooterPosition && Timer::GetPPCTimestamp() < autonStartTime + autonWinchTimeout )
			shooter.runWinch();
		else
			shooter.stopWinch();
		break;
	case 3:
		///needs testing
		if(!hasShot) {
			shooter.releaseWinch();
			hasShot = true;
		}
		if(!hasGathered) {
			gatherer.setArmAngle(gatherer.FORWARD_POSITION);
			gatherer.rollerControl(1.0);
			if(driveForward(2,autonSpeed, autonStartTime + autonDriveTimeout) == hasReachedDestination) {
				if(Timer::GetPPCTimestamp() > autonStartTime + 3.0)
					hasGathered = true;
			}
		} else {
			driveForward(autonDistance,autonSpeed, autonStartTime + autonDriveTimeout);
		}
		if(shooterPot.GetVoltage() > autonShooterPosition)
			shooter.runWinch();
		else
			shooter.stopWinch();		break;
	}
}
void Robot::TeleopInit() {
	leftEncoder.SetDistancePerPulse(1.0);
	rightEncoder.SetDistancePerPulse(1.0);
	timeOfLastShot = Timer::GetPPCTimestamp();
	gatherer.setPIDEnabled(false);
	gatherer.setArmAngle(gatherer.FORWARD_POSITION);
}
void Robot::TeleopPeriodic() {
	static bool TECH_BACK_PREVIOUS = false, TECH_BACK_CURRENT;
	static bool TECH_YBUTTON_PREVIOUS = false, TECH_YBUTTON_CURRENT;
	static bool TECH_XBUTTON_PREVIOUS = false, TECH_XBUTTON_CURRENT;
	
	drive.takeSpeedSample();
	drive.shiftAutomatically();
	float moveValue = stick.GetAxis(Joystick::kYAxis);
	float rotateValue = stick.GetAxis(Joystick::kTwistAxis);
	TECH_BACK_CURRENT = tech.GetRawButton(XBOX::BACK);
	if(TECH_BACK_CURRENT && !TECH_BACK_PREVIOUS) {
		shooter.toggleAutoLoad();
	}
	TECH_BACK_PREVIOUS = TECH_BACK_CURRENT;
	//deadband for joystick
	if(fabs(moveValue) < deadbandWidth)moveValue = 0.0;
	if(fabs(rotateValue) < deadbandWidth)rotateValue = 0.0;
	//drive code
	drive.ArcadeDrive(moveValue,rotateValue);
	//run roller
	if(stick.GetRawButton(XBOX::RBUMPER)||tech.GetRawButton(XBOX::RBUMPER))
		gatherer.rollerControl(1);
	else if (stick.GetRawButton(XBOX::LBUMPER)||tech.GetRawButton(XBOX::LBUMPER)) //run roller backward
		gatherer.rollerControl(-1);
	else
		gatherer.rollerControl(0);
	//catapault
	float secondsSinceLastShot = Timer::GetPPCTimestamp() - timeOfLastShot;
	if(stick.GetRawAxis(Joystick::kTwistAxis) < -0.8) { //right trigger = shoot
		shooter.stopWinch();
		shooter.releaseWinch();
		timeOfLastShot = Timer::GetPPCTimestamp();
	}else if(((shooter.autoLoad && shooterPot.GetVoltage() > shooter.POT_MIN)
			|| stick.GetRawAxis(Joystick::kTwistAxis) > 0.8) &&  secondsSinceLastShot > 0.5) {//left trigger
		if(!shooter.isWinchEngaged())
			shooter.engageWinch();
		shooter.runWinch();
	}else {
		shooter.stopWinch();
	}
	float gatherControl = tech.GetRawAxis(Joystick::kTwistAxis);
	if(fabs(gatherControl) > 0.03) {
		//should pid control be disabled? or should setpoint be updated
		arm.Set(gatherControl * 0.5); //Manual Gatherer control
	} else if(gatherer.isPIDEnabled()){
		if(stick.GetRawButton(XBOX::BBUTTON)) {
			gatherer.setArmAngle(gatherer.FORWARD_POSITION);
		} else if(stick.GetRawButton(XBOX::ABUTTON)) {
			gatherer.setArmAngle(gatherer.DOWN_POSITION);			
		} else if(stick.GetRawButton(XBOX::YBUTTON)) {
			gatherer.setArmAngle(gatherer.UP_POSITION);			
		} else if(stick.GetRawButton(XBOX::XBUTTON)) {
			gatherer.setArmAngle(gatherer.BACKWARD_POSITION);
		}
	} else {
		arm.Set(0.0);
	}
	//zeroing gatherer potentiometer values
	if(tech.GetRawButton(XBOX::BBUTTON)) {
		gatherer.setDownPosition(gathererPot.GetVoltage());
	}
	
	//disabling PID gatherer control
	TECH_YBUTTON_CURRENT = tech.GetRawButton(XBOX::YBUTTON);
	if(TECH_YBUTTON_CURRENT && !TECH_YBUTTON_PREVIOUS) {
		gatherer.togglePIDEnabled();
	}
	TECH_YBUTTON_PREVIOUS = TECH_YBUTTON_CURRENT;
	
	//toggle autoshift enable
	TECH_XBUTTON_CURRENT = tech.GetRawButton(XBOX::XBUTTON);
	if(TECH_XBUTTON_CURRENT && !TECH_XBUTTON_PREVIOUS) {
		drive.toogleAutoShiftEnable();
	}
	PrintInfoToSmartDashboard();
}
};
START_ROBOT_CLASS(Robot);
