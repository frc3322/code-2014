#include "WPILib.h"
#include "../xbox.h"
#include "SmartDashboard/SmartDashboard.h"
#include "DriveTrain.h"
#include "TripleSpeedController.h"
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
	Joystick driver;	//both are xbox controllers
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
	double timeOfLastShot;
	double autonWinchTimeout;
	//atuon variables
	bool hasGoneForward;
	bool hasShot;
	double timeOfShot;
	bool readyForSecondBall;
	double timeDrawnBack;
	double timeOfStart;
	double timeReachedDestination;
	bool TECH_RIGHTBUMPER_PREVIOUS, TECH_RIGHTBUMPER_CURRENT, TECH_START_PREVIOUS, TECH_START_CURRENT;
	bool TECH_LEFTBUMPER_PREVIOUS, TECH_LEFTBUMPER_CURRENT;

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
		driver(1),tech(2),
		gathererPot(2), shooterPot(1),
		armController(-1.0,0.0,0.0,&gathererPot,&arm), //should get PID values from smartdashboard for the purpose of testing
		gatherer(&roller,&gathererPot,&armController),
		shooter(&winch, &shooterPot, &trigger),
		deadbandWidth(0.01),
#if ROBOT==COMP
		P(-6.00), I(-0.01),D(-4.00),
#else
		P(4.0), I(0.01), D(4.0),
#endif
		autonMode(2), autonStartTime(0.0), autonDistance(16.0), autonSpeed(0.8), autonDriveTimeout(10.0), autonWinchTimeout(5.0),
		timeReachedDestination(0.0)
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
		SmartDashboard::PutNumber("Shooter Pot Fire Position", shooter.SHOOT_POSITION);
		SmartDashboard::PutNumber("Gatherer Down Position", gatherer.DOWN_POSITION);
		SmartDashboard::PutNumber("Gatherer Forward Position", gatherer.FORWARD_POSITION);
		SmartDashboard::PutNumber("Gatherer Up Position", gatherer.UP_POSITION);
		SmartDashboard::PutNumber("Gatherer Back Position", gatherer.BACKWARD_POSITION);

		char label[3];
		label[0] = '0';
		label[1] = ')';
		label[2] = '\0';
		const char* descriptions[] = {
				"Drive to low goal",
				"Drive to low goal and reverse gatherer to feed ball into low goal",
				"Drive, pull down shooter, fire"
				"Drive, shoot, gather, shoot again"
		};
		for(unsigned int i = 0; i < sizeof(descriptions)/sizeof(const char*); i++){
			SmartDashboard::PutString(label,descriptions[i]);
			label[0] = label[0] < '9' ? label[0] + 1 : '0';
		}
		SmartDashboard::PutNumber("autonStartTime",autonStartTime);
		SmartDashboard::PutNumber("autonDistance",autonDistance);
		SmartDashboard::PutNumber("autonSpeed",autonSpeed);
		SmartDashboard::PutNumber("autonDriveTimeout",autonDriveTimeout);
		SmartDashboard::PutNumber("autonWinchTimeout",autonWinchTimeout);
	}
	void Robot::PrintInfoToSmartDashboard() {
		SmartDashboard::PutNumber("left  encoder",leftEncoder.GetDistance());
		SmartDashboard::PutNumber("right encoder",rightEncoder.GetDistance());	
		//drive.setAutoShiftEnable(SmartDashboard::GetBoolean("Is auto shift enabled"));
		SmartDashboard::PutBoolean("auto shift enabled", drive.isAutoShiftEnabled());
		SmartDashboard::PutNumber("Shooter potentiometer", shooterPot.GetVoltage());
		SmartDashboard::PutBoolean("high gear", drive.isInHighGear());
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
		autonMode = abs((int)SmartDashboard::GetNumber("auton mode"));
		SmartDashboard::PutNumber("PPC time", Timer::GetPPCTimestamp());
		SmartDashboard::PutBoolean("auto load catapult",shooter.autoLoad);
		SmartDashboard::PutNumber("tech trigger axis",tech.GetRawAxis(Joystick::kTwistAxis));
		SmartDashboard::PutNumber("autonStartTime",autonStartTime);
		autonDistance = SmartDashboard::GetNumber("autonDistance");
		autonSpeed = SmartDashboard::GetNumber("autonSpeed");
		autonDriveTimeout = fabs(SmartDashboard::GetNumber("autonDriveTimeout"));
		autonWinchTimeout = fabs(SmartDashboard::GetNumber("autonWinchTimeout"));
		shooter.SHOOT_POSITION = SmartDashboard::GetNumber("Shooter Pot Fire Position");
		gatherer.DOWN_POSITION = SmartDashboard::GetNumber("Gatherer Down Position");
		gatherer.FORWARD_POSITION = SmartDashboard::GetNumber("Gatherer Forward Position");
		gatherer.UP_POSITION = SmartDashboard::GetNumber("Gatherer Up Position");
		gatherer.BACKWARD_POSITION = SmartDashboard::GetNumber("Gatherer Back Position");
		
		SmartDashboard::PutNumber("left  encoder speed",leftEncoder.GetRate());
		SmartDashboard::PutNumber("right encoder speed",rightEncoder.GetRate());

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
		shooter.engageWinch();
		gatherer.setPIDEnabled(autonMode == 4);
		hasGoneForward = false;
		hasShot = false;
		timeOfShot = 0.0;
		readyForSecondBall = false;
		timeDrawnBack = 0.0;
		timeOfStart = Timer::GetPPCTimestamp();
		timeReachedDestination = 0.0;
		compressor.Start();
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
	void Robot::drawBackShooter() {
		if(!shooter.isDrawnBack()) { 
			shooter.engageWinch();
			shooter.runWinch();
		}
		else {
			//should have a check!
			shooter.stopWinch();
		}
	}
	void Robot::auton4() {
		//reduced wait times, needs testing
		if(!hasGoneForward) {
			drawBackShooter();
			gatherer.setArmAngle(gatherer.FORWARD_POSITION);
			hasGoneForward = driveForward(autonDistance,autonSpeed, autonStartTime + autonDriveTimeout);
		}
		else if(!hasShot || Timer::GetPPCTimestamp() < timeOfShot + 0.4) {
			shooter.releaseWinch();
			if(!hasShot) {
				hasShot = true;
				timeOfShot = Timer::GetPPCTimestamp();
			}
		}
		else if (!readyForSecondBall) {
			drawBackShooter();
			if(Timer::GetPPCTimestamp() > timeOfShot + 0.5) {
				//opposite on comp
				gatherer.rollerControl(1.0);
			}
			if(shooter.isDrawnBack()) {
				readyForSecondBall = true;
				timeDrawnBack = Timer::GetPPCTimestamp();
			}
		}
		else if(Timer::GetPPCTimestamp() > timeDrawnBack + 1.5) {
			shooter.releaseWinch();
			//untested as of 3/22 5:30pm
			shooter.stopWinch();
		}
	}
	void Robot::AutonomousPeriodic() {
		PrintInfoToSmartDashboard();
		switch (autonMode) {
		case 0:	//Drive to low goal
			driveForward(autonDistance, autonSpeed, autonStartTime + autonDriveTimeout);
			break;
		case 1:	//Drive to low goal and reverse gatherer to feed ball into low goal
			gatherer.setArmAngle(gatherer.BACKWARD_POSITION);		//pull back gatherer arm
			if(driveForward(autonDistance, autonSpeed, autonStartTime + autonDriveTimeout))//drive forward to low goal - returns true at destination
				gatherer.rollerControl(-1);		//run rollers to put ball in low goal
			break;
		case 2:
			if(driveForward(autonDistance, autonSpeed, autonStartTime + autonDriveTimeout)) {
				if(timeReachedDestination < 1.0) {
					timeReachedDestination = Timer::GetPPCTimestamp();
				}
				
				double elapsedTimeAtDestination = Timer::GetPPCTimestamp() - timeReachedDestination;
				if(elapsedTimeAtDestination > 2.0 && shooter.isDrawnBack() && shooter.isWinchEngaged() && timeOfShot < 1.0) {
					shooter.releaseWinch();
					timeOfShot = Timer::GetPPCTimestamp();
				}
			}
			if(!shooter.isDrawnBack()) {
				if(!shooter.isWinchEngaged() && (timeOfShot < 1.0 || Timer::GetPPCTimestamp() > timeOfShot + 0.5)) {
					shooter.engageWinch();
				}
				shooter.runWinch();
			}
			else
				shooter.stopWinch();
			break;
		case 4:
			auton4();
			break;
		}
	}
	void Robot::TeleopInit() {
		leftEncoder.SetDistancePerPulse(1.0);
		rightEncoder.SetDistancePerPulse(1.0);
		timeOfLastShot = Timer::GetPPCTimestamp();
		gatherer.setPIDEnabled(false);
		gatherer.setArmAngle(gatherer.FORWARD_POSITION);
		TECH_RIGHTBUMPER_PREVIOUS = false;
		TECH_START_PREVIOUS = false;
		TECH_LEFTBUMPER_PREVIOUS = false;
		compressor.Start();
		shooter.shooterInit();
	}
	void Robot::TeleopPeriodic() {

		drive.takeSpeedSample();
		drive.shiftAutomatically();
		
		float moveValue = driver.GetAxis(Joystick::kYAxis);
		if(fabs(moveValue) < deadbandWidth)moveValue = 0.0;
		float rotateValue = driver.GetAxis(Joystick::kTwistAxis);
		if(fabs(rotateValue) < deadbandWidth)rotateValue = 0.0;
		//drive code
		drive.ArcadeDrive(moveValue,rotateValue);
		
		
		TECH_RIGHTBUMPER_CURRENT = tech.GetRawButton(XBOX::RBUMPER);
		if(TECH_RIGHTBUMPER_CURRENT && !TECH_RIGHTBUMPER_PREVIOUS) {
			shooter.toggleAutoLoad();
		}
		TECH_RIGHTBUMPER_PREVIOUS = TECH_RIGHTBUMPER_CURRENT;


		//run roller
		if(driver.GetRawButton(XBOX::RBUMPER) || tech.GetRawAxis(Joystick::kDefaultYAxis) > .8 )
			gatherer.rollerControl(1);
		else if (driver.GetRawButton(XBOX::LBUMPER) || tech.GetRawAxis(Joystick::kDefaultYAxis) < -.8 ) //run roller backward
			gatherer.rollerControl(-1);
		else
			gatherer.rollerControl(0);
		
		//catapault
		float secondsSinceLastShot = Timer::GetPPCTimestamp() - timeOfLastShot;
		if(driver.GetRawAxis(Joystick::kTwistAxis) < -0.8) { //right trigger = shoot
			shooter.stopWinch();
			shooter.releaseWinch();
			timeOfLastShot = Timer::GetPPCTimestamp();
		}
		else if((shooter.autoLoad && !shooter.isDrawnBack()) || (driver.GetRawButton(XBOX::BACK) && secondsSinceLastShot > 0.5)) {//left trigger
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
			if(driver.GetRawButton(XBOX::BBUTTON) || (tech.GetRawButton(XBOX::BBUTTON)) ) {
				gatherer.setArmAngle(gatherer.FORWARD_POSITION);
			} else if(driver.GetRawButton(XBOX::ABUTTON) || (tech.GetRawButton(XBOX::ABUTTON))) {
				gatherer.setArmAngle(gatherer.DOWN_POSITION);			
			}else if(driver.GetRawButton(XBOX::YBUTTON) || (tech.GetRawButton(XBOX::YBUTTON))) {
				gatherer.setArmAngle(gatherer.UP_POSITION);			
			} else if(driver.GetRawButton(XBOX::XBUTTON) || (tech.GetRawButton(XBOX::XBUTTON))) {
				gatherer.setArmAngle(gatherer.BACKWARD_POSITION);
			}
		} else {
			arm.Set(0.0);
		}
		//zeroing gatherer potentiometer values
		//would be good to only do this if PID is enabled
		if(tech.GetRawButton(XBOX::BACK)) {
			gatherer.setDownPosition(gathererPot.GetVoltage());
		}

		//disabling PID gatherer control
		TECH_START_CURRENT = tech.GetRawButton(XBOX::START);
		if(TECH_START_CURRENT && !TECH_START_PREVIOUS) {
			gatherer.togglePIDEnabled();
			//we could set a default position
		}
		TECH_START_PREVIOUS = TECH_START_CURRENT;

		//toggle autoshift enable
		TECH_LEFTBUMPER_CURRENT = tech.GetRawButton(XBOX::LBUMPER);
		if(TECH_LEFTBUMPER_CURRENT && !TECH_LEFTBUMPER_PREVIOUS) {
			drive.toogleAutoShiftEnable();
		}
		TECH_LEFTBUMPER_PREVIOUS = TECH_LEFTBUMPER_CURRENT;
		PrintInfoToSmartDashboard();
	}
};
START_ROBOT_CLASS(Robot);
