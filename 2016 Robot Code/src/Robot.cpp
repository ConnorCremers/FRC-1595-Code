#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <CANTalon.h>
#include <Joystick.h>
#include <Solenoid.h>
#include <DoubleSolenoid.h>

#include "AHRS.h"
#include "MATH.h"

using namespace frc;



class Robot: public IterativeRobot {
public:

	void RobotInit() {
		table = NetworkTable::GetTable("datatable");
		lw = LiveWindow::GetInstance();
		try {
			ahrs = new AHRS(SPI::Port::kMXP);
		} catch (std::exception ex ) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		if ( ahrs ) {
			LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
		}
		//0,1000,-597
		pot = new AnalogPotentiometer(0, 1000, -600);

		intakeLiftControl = new PIDController(kPI, kII, kDI, pot, &intakeLiftPID);
		intakeLiftControl->SetContinuous(false);

		angleControl = new PIDController(kPG, kIG, kDG, ahrs, &anglePID);
		angleControl->SetContinuous(true);
		angleControl->SetInputRange(-180, 180);

		disControl= new PIDController(kPD,kID,kDD,lEnc,&disPID);
		disControl->SetContinuous(false);


		//Vision
		vision= NetworkTable::GetTable("visionData");
		vision->PutNumber("COG_X",0);
		vision->PutNumber("COG_Y",0);
		vision->PutNumber("RUN_TIME",0);
		//Vision

	  //Drive stuff
		lDrive2.SetControlMode(CANSpeedController::kFollower);	//so that they take input of other SRXs
		rDrive2.SetControlMode(CANSpeedController::kFollower);
		lDrive2.Set(2);	//sets it to the controller on port 0
		rDrive2.Set(0); //sets it to the controller on port 2
		lDrive1.SetInverted(true);
		lEnc->SetDistancePerPulse(wheelDia*M_PI/128);
	    rEnc->SetDistancePerPulse(wheelDia*M_PI/128);
		//setting motor control modes

		//Shooter
		lowerFly.SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		upperFly.SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		lowerFly.SetControlMode(CANTalon::kSpeed);
		upperFly.SetControlMode(CANTalon::kSpeed);
		lowerFly.SetPID(kPF,kIF,kDF,kFF);
		upperFly.SetPID(kPF,kIF,kDF,kFF);
		upperFly.SetSensorDirection(true);
		lowerFly.SetSensorDirection(true);
		upperFly.SetIzone(iF);
		lowerFly.SetIzone(iF);
		shooterLift.SetControlMode(CANTalon::kPosition);
		shooterLift.SetFeedbackDevice(CANTalon::QuadEncoder);
		shooterLift.SetPID(.5, 0, 30);
		shooterLift.ConfigForwardLimit(198352);
		shooterLift.ConfigReverseLimit(0);
		ballControl.SetInverted(true);

		intakeLift.SetInverted(true);

	}
	void AutonomousInit() override {
	}

	void AutonomousPeriodic() {
	}

	void TeleopInit() {
		intakeLiftControl->SetPID(kPI,kII,kDI);
		intakeLiftControl->SetOutputRange(-1,1);
		intakeLiftControl->Enable();

		angleControl->SetPID(kPGs,kIGs,kDGs);
		angleControl->SetOutputRange(minGs,maxGs);
		angleControl->Enable();

		upperFly.SetPID(kPF,kIF,kDF,kFF);
		lowerFly.SetPID(kPF,kIF,kDF,kFF);
		upperFlySetSpeed=0;
		lowerFlySetSpeed=0;

		shooterLift.SetPosition(0);

		highGear=true;

		upperFlyTimer.Start();
		upperFlyTimer.Reset();
		lowerFlyTimer.Start();
		lowerFlyTimer.Reset();

		visionSetPointOnce=false;
	}

	void TeleopPeriodic() {
		visionRunTime=vision->GetNumber("RUN_TIME",0);

		throttle= -dr.GetRawAxis(1);
		turn= dr.GetRawAxis(2);
		turnButton= dr.GetRawButton(6);
		aim=dr.GetRawButton(5);
		visionAim=dr.GetRawButton(7);
		if(dr.GetRawButton(11)){
			highGear=true;
		}
		if(dr.GetRawButton(12)){
			highGear=false;
		}
			//op
		if(op.GetPOV()==0){
			shooterLiftPIDEnabled=true;
			shooterHighD=true;
			shooterHighB=false;
			shooterLow=false;
		}
		if(op.GetPOV()==270){
			shooterLiftPIDEnabled=true;
			shooterHighD=false;
			shooterHighB=true;
			shooterLow=false;
		}
		if(op.GetPOV()==180){
			shooterLiftPIDEnabled=true;
			shooterHighD=false;
			shooterHighB=false;
			shooterLow=true;
		}
		shooterLiftManualPow= -op.GetRawAxis(1)*0.3;
		shooterLiftManual=op.GetRawButton(11);
		shooterFlyPre=op.GetRawButton(5);
		shooterFlyFire=op.GetRawButton(6);

		if(op.GetRawButton(2)){
			intakeLiftPIDEnabled=true;
			intakeLiftLow=true;
			intakeLiftCDF=false;
			intakeLiftHigh=false;
		}
		if(op.GetRawButton(3)){
			intakeLiftPIDEnabled=true;
			intakeLiftLow=false;
			intakeLiftCDF=true;
			intakeLiftHigh=false;
		}
		if(op.GetRawButton(4)){
			intakeLiftPIDEnabled=true;
			intakeLiftLow=false;
			intakeLiftCDF=false;
			intakeLiftHigh=true;
		}
		if(op.GetRawButton(1)){
			intakeLiftPIDEnabled=true;
			intaking=true;
		}
		else{
			intaking=false;
		}
		if(op.GetRawButton(12)){
			intakeLiftPIDEnabled=false;
			intakeLiftManual=true;
		}
		else{
			intakeLiftManual=false;
		}

		intakeLiftManualPow= -op.GetRawAxis(3)*0.6;






		//Driver Code
		//Cheesy Drive
/*
		if(testPID.GetRawButton(1)){
			angleControl->SetSetpoint(testPID.GetRawAxis(1) * 180);
			SmartDashboard::PutNumber("Turn to setpoint", testPID.GetRawAxis(1) * 180);
			angleControlOutput = angleControl->Get();
			lDrive1.Set(angleControlOutput);
			rDrive1.Set(-angleControlOutput);
		}
		else if(testPID.GetRawButton(3)){
			if(!setPointOnce){
				setPointOnce = true;
				ahrs->ZeroYaw();
			}
			angleControl->SetSetpoint(0);
			angleControlOutput = angleControl->Get();
			lPow =  .75 + angleControlOutput;
			rPow =  .75 - angleControlOutput;
			lDrive1.Set(lPow);
			rDrive1.Set(rPow);
		}
		else*/
		if(visionAim){
			if(!visionSetPointOnce){
				visionX=vision->GetNumber("COG_X",0);
				visionY=vision->GetNumber("COG_Y",0);
				visionSetPointOnce = true;
				visionSetPoint = ahrs->GetYaw()-(visionX*-.095 + visionMidPoint*0.095);
				visionSetPointY = shooterLift.GetPosition()/1983.52 - 0.0897*(205-visionY);
			}
			angleControl->SetSetpoint(visionSetPoint);
			angleControlOutput = angleControl->Get();
			turn=angleControlOutput;
		}
		else{
			visionSetPointOnce=false;
			if(aim){
				turn *= 0.5;
			}
			else if(!turnButton) {
				turn *= (turnSpeedGain * fabs(throttle));
			}
		}


		t_left = throttle + turn;
		t_right = throttle - turn;
		if (t_left > 1.0) {

			skimLeft = -((t_left - 1.0) * turnSkimGain);
		}
		else if (t_left < -1.0) {
			skimLeft = -((t_left + 1.0) * turnSkimGain);
		}
		else {
			skimLeft = 0;
		}
		if (t_right > 1.0) {
			skimRight = -((t_right - 1.0) * turnSkimGain);
		}
		else if (t_right < -1.0) {
			skimRight = -((t_right + 1.0) * turnSkimGain);
		}
		else {
			skimRight = 0;
		}
		left = t_left + skimLeft;
		right = t_right + skimRight;
		if (fabs(left) < 0.02) {
			left = 0;
		}
		if (fabs(right) < 0.02)
		{
			right = 0;
		}
		lPow = left;
		rPow = right;
		lDrive1.Set(lPow);
		rDrive1.Set(rPow);

		if(visionAim){
			shifter.Set(DoubleSolenoid::kForward);
		}
		else if(aim){
			shifter.Set(DoubleSolenoid::kForward);
		}
		else if(highGear){
			shifter.Set(DoubleSolenoid::kReverse);
		}
		else{
			shifter.Set(DoubleSolenoid::kForward);
		}



		//Operator Code

		//Shooter

		if(intaking){
			shooterLiftSetPos=0;
		}
		else{
			if(shooterHighD){
				if(visionAim){
					shooterLiftSetPos=visionSetPointY;
				}
				else{
					shooterLiftSetPos=shooterLiftSetPosTry;
				}
			}
			if(shooterHighB){
				shooterLiftSetPos=95;
			}
			if(shooterLow){
				if(shooterFlyPre){
					shooterLiftSetPos=40;
				}
				else{
					shooterLiftSetPos=0;
				}

			}
		}

		//Intake
		if(intaking){
			intakeLiftPIDEnabled=true;
			intakeLiftSetAngle=20;
		}
		else{
			if(intakeLiftHigh){
				intakeLiftSetAngle=90;
			}
			if(intakeLiftCDF){
				intakeLiftSetAngle=35;
			}
			if(intakeLiftLow){
				intakeLiftSetAngle=0;
			}
		}

		//FlyWheel
		if(intaking){
			if(boulderIn.Get()){
				upperFlySetSpeed=-3200;
				lowerFlySetSpeed=-3200;
				ballControlSetPow= -1;
				intakeRollerSetPow=-1;
			}
			else{
				upperFlySetSpeed=0;
				lowerFlySetSpeed=0;
				ballControlSetPow= -0.5;
				intakeRollerSetPow=0;
			}
		}
		else if(shooterFlyPre)
		{

			if(shooterHighD){
				upperFlySetSpeed=upperFlySetSpeedTry;
				lowerFlySetSpeed=lowerFlySetSpeedTry;
			}
			if(shooterHighB){
				upperFlySetSpeed=4200;
				lowerFlySetSpeed=3500;
			}
			if(shooterLow){
				upperFlySetSpeed=2500;
				lowerFlySetSpeed=2500;
			}

			if(shooterFlyFire){
		\
		ballControlSetPow= 1;
			}
			else{
				ballControlSetPow= -0.2;
			}
		}
		else{
			upperFlySetSpeed=0;
			lowerFlySetSpeed=0;
			ballControlSetPow=0;
			intakeRollerSetPow=0;
		}









		//Actuating
		//Anti-Collision Algorithm
		shooterLiftPos=shooterLift.GetPosition()/1983.52;
		intakeLiftAngle=pot->Get();
		if(shooterLiftPos<10 && shooterLiftSetPos>10){
			if(intakeLiftSetAngle>45){
				intakeLiftSetAngle=45;
			}
			else{}
		}
		if(shooterLiftPos>10 && shooterLiftPos<50){
			if(intakeLiftSetAngle>45){
				intakeLiftSetAngle=45;
			}
			else{}
		}
		if(shooterLiftPos>50 && shooterLiftSetPos<50){
			if(intakeLiftSetAngle>45){
				intakeLiftSetAngle=45;
			}
			else{}
		}
		if(intakeLiftAngle>50 && shooterLiftPos>8 && shooterLiftPos<52){
			if(shooterLiftPos<30){
				shooterLiftSetPos=10;
			}
			if(shooterLiftPos>=30){
				shooterLiftSetPos=50;
			}
		}
		else{
		}


		//IntakeBrake

		//ShooterLift



		if(shooterLiftManual){
			shooterLiftPIDEnabled=false;
			shooterLift.Set(shooterLiftManualPow);
		}
		else if(shooterLiftPIDEnabled){
			if(shooterLiftSetPos-shooterLift.GetPosition()/1983.52>5){
				shooterLift.SetPID(.5, 0, 30);
			}
			if(shooterLiftSetPos-shooterLift.GetPosition()/1983.52<-5){
				shooterLift.SetPID(0.1, 0, 30);
			}
			shooterLift.Set(shooterLiftSetPos*1983.52);
		}
		else{
			shooterLift.Set(0);
		}

		if(shooterLiftPIDEnabled){
			shooterLift.SetControlMode(CANTalon::kPosition);
		}
		else{
			shooterLift.SetControlMode(CANTalon::kPercentVbus);
		}








		//IntakeLift

		if(intakeLiftPIDEnabled){
			intakeLiftControl->Enable();
		}
		else{
			intakeLiftControl->Disable();
		}

		if(intakeLiftManual){
			intakeLift.Set(intakeLiftManualPow);
			intakeBrake.Set(DoubleSolenoid::kReverse);//Release Brake
		}
		else if(intakeLiftPIDEnabled){
			//Added

			//Added
			intakeLiftControl->SetSetpoint(intakeLiftSetAngle);
			intakeLift.Set(intakeLiftControl->Get());
			if(abs(intakeLiftSetAngle-intakeLiftAngle)<3){
				intakeBrake.Set(DoubleSolenoid::kForward);
			}
			else if(abs(intakeLiftSetAngle-intakeLiftSetAnglePrev)>4){
				intakeBrake.Set(DoubleSolenoid::kReverse);
			}
		}
		else{
			intakeLiftSetAngle=96;
			intakeLift.Set(0);
			intakeBrake.Set(DoubleSolenoid::kForward);//Brake
		}
		intakeLiftSetAnglePrev=intakeLiftSetAngle;


			if(upperFlySetSpeed==0){
				upperFly.Disable();
			}
			else{
				upperFly.Enable();
				upperFly.Set(upperFlySetSpeed);
			}
			if(lowerFlySetSpeed==0){
				lowerFly.Disable();
			}
			else{
				lowerFly.Enable();
				lowerFly.Set(lowerFlySetSpeed);
			}


			/*
			if(upperFlySetSpeed==0){
				upperFly.Disable();
			}
			else{
				upperFly.Enable();
				upperFly.Set(upperFlySetSpeed);
			}
			if(lowerFlySetSpeed==0){
				lowerFly.Disable();
			}
			else{
				lowerFly.Enable();
				lowerFly.Set(lowerFlySetSpeed);
			}
			*/

		//change


		ballControl.Set(ballControlSetPow);
		intakeRoller.Set(intakeRollerSetPow);


		//Indicators
		if(abs(upperFly.GetClosedLoopError()/6.8267)<10){
		}
		else{
			upperFlyTimer.Reset();
		}
		if(abs(lowerFly.GetClosedLoopError()/6.8267)<10){
		}
		else{
			lowerFlyTimer.Reset();
		}
		if(upperFlyTimer.Get()>0.5 && lowerFlyTimer.Get()>0.5){
			SmartDashboard::PutBoolean("onTarget",true);
			indicatorController.Set(true);
		}
		else{
			SmartDashboard::PutBoolean("onTarget",false);
			indicatorController.Set(false);
		}
	}

	void TestPeriodic() {
	}

private:
	//Vision
	std::shared_ptr<NetworkTable> vision;
	double visionX=0,visionY=0,visionSetPoint,visionSetPointY,visionRunTime=0,visionLastRunTime=0,visionMidPoint=318;
	Solenoid lightController { 0 , 4 },indicatorController { 0 , 3 };
	Timer visionComTimer;
	//Vision


	//GYRO STUFF--THE FOLLOWING IS FOR THE GYRO
	std::shared_ptr<NetworkTable> table;
    AHRS *ahrs;
    LiveWindow *lw;
    int autoLoopCounter = 0;

	//DRIVE ITEMS--THE FOLLOWING ONLY HAS TO DO WITH DRIVE TRAIN ITEMS
	CANTalon lDrive1 { 2 }, rDrive1{0}; //the controlled two motors on the drivetrain
	CANTalon lDrive2{3}, rDrive2{1}; //The two motors which will be set as followers
	Encoder *lEnc = new Encoder(0,1,true,Encoder::EncodingType::k4X);
	Encoder *rEnc = new Encoder(2,3,false, Encoder::EncodingType::k4X);
	DoubleSolenoid shifter{1,0,1}; //the solenoid which will shift the gearing
	DoubleSolenoid PTO{0,0,7}; //solenoid for PTO
	float throttle, turn	//adjusted inputs from joysticks
	,lPow ,rPow;	//powers for drive train
	float t_left, skimLeft, left;
	float t_right, skimRight, right;
	bool turnButton,highGear,aim,visionAim,visionSetPointOnce;
	float turnSpeedGain=0.75,turnSkimGain=0.7;

	//SHOOTER ITEMS--THE FOLLOWING ONLY HAS TO DO WITH THE SHOOTER
	Timer upperFlyTimer,lowerFlyTimer;
	double kPF=0.11,kIF=0.0006,kDF=5,kFF=0.0322,iF=600;
	CANTalon lowerFly{14}, upperFly{13}; //the two flywheels
	CANTalon ballControl{12}; //thing to push ball to flywheels
	CANTalon shooterLift{15}; //the lift for the shooter
	bool shooterLiftInit=false,shooterLiftBrake=false;
	DigitalInput boulderIn{4}; //the switch of whether or not ball is in
	DoubleSolenoid shooterBrake{0,1,6}; //brake on the shooter lift
	float shooterLiftManualPow,lowerFlySetSpeed,upperFlySetSpeed,ballControlSetPow;
	float shooterLiftSetPos,shooterLiftPos;
	bool shooterLow,shooterHighB,shooterHighD,shooterFlyPre,shooterFlyFire,shooterLiftManual,shooterLiftPIDEnabled=true;

	//INTAKE ITEMS--THE FOLLOWING ONLY HAS TO DO WITH THE INTAKE
	Victor intakeRoller{10}; //motor running the intake
	float intakeRollerSetPow;
	CANTalon intakeLift{4}; //motor which raises/lowers intake
	float intakeLiftSetAngle,intakeLiftManualPow,intakeLiftAngle,intakeLiftSetAnglePrev;
	bool intakeLiftLow,intakeLiftCDF,intakeLiftHigh,intaking,intakeLiftManual,intakeLiftPIDEnabled=true;
	float kPI=0.03, kII=0, kDI=0, minI=-1, maxI=1;
	Potentiometer *pot;
	Talon intakeLiftPID{2};
	PIDController *intakeLiftControl;
	DoubleSolenoid intakeBrake{0,2,5};



	//Joystick
	Joystick dr{0}, op{1},tr{2}; //joysticks for driver and operator


	//PID
	double kPG=0.025, kIG=0.005, kDG=0.06, minG=-0.75, maxG=0.75;
	double kPGs=0.08, kIGs=0, kDGs=0.01, minGs=-0.5, maxGs=0.5;
	double kPD=0.02, kID=0, kDD=0, minD=-1, maxD=1;
	Talon anglePID{0},disPID{1};
	PIDController *angleControl,*disControl;

	float angleControlOutput;

	Compressor *c = new Compressor(1);


	float upperFlySetSpeedTry=4400,lowerFlySetSpeedTry=4400,shooterLiftSetPosTry=77,intakeSetAngleTry;




	//Autonomous
	Timer autoTimer,angleTimer,shooterLiftTimer;
	int autoStage=0,autoDefenseSelected=1,autoPositionSelected=1;
	bool autoShoot=true, intakeFinished=false;
	float wheelDia=7.65;
	double crossingTime=3.25, autoSetAngle=0, autoForwardDis=16,autoIntakeLowerStage=0,autoDrivingForwardAngle;


};

START_ROBOT_CLASS(Robot)
