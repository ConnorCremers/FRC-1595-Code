#include "WPILib.h"
#include "Functions.h"
#include "AHRS.h"
#include <iostream>
class Robot: public SampleRobot
{
	//James Added for SmartDashboard
	//Preferences *prefs;
	double AutoSetDis,kP,kI,kD;

	//GYRO STUFF--THE FOLLOWING IS FOR THE GYRO
	std::shared_ptr<NetworkTable> table;
    AHRS *ahrs;
    LiveWindow *lw;
    int autoLoopCounter;

	//DRIVE ITEMS--THE FOLLOWING ONLY HAS TO DO WITH DRIVE TRAIN ITEMS
	CANTalon lDrive1, rDrive1; //the controlled two motors on the drivetrain
	CANTalon lDrive2, rDrive2; //The two motors which will be set as followers
	DoubleSolenoid shifter; //the solenoid which will shift the gearing
	bool manualLift; //for knowing if it can go back to auto lifting/lowering wheels
	float throttle, turn	//adjusted inputs from joysticks
	,lPow ,rPow;	//powers for drive train
	double lDis, rDis;

	//SHOOTER ITEMS--THE FOLLOWING ONLY HAS TO DO WITH THE SHOOTER
	CANTalon lowerFly, upperFly; //the two flywheels
	CANTalon ballControl; //thing to push ball to flywheels
	CANTalon shooterLift;
	float wheelSpeed; //how fast to move wheels
	bool shootIntake;
	Timer speedTime;
	float lowerSpeed = 0, upperSpeed = 0;
	bool run=false, shotReset;
	int position = 0;


	//INTAKE ITEMS--THE FOLLOWING ONLY HAS TO DO WITH THE INTAKE
	CANTalon intakeRoller; //motor running the intake
	CANTalon intakeLift; //motor which raises/lowers intake
	int intakeVal = 0; //position to set the lift
	DigitalInput boulderIn; //a switch to signal that the boulder is fully entered
	float ballControlVal;
	bool set;

	Joystick driver, operater; //joysticks for driver and operator (i know i misspelled it, operator is a storage type)
	int autoChooser = 0;
	bool lowerIntake;
	Timer autoTime;
	float curErr, avgDst;
	int toDrive = 16900; //needs to be about 207 inches
	bool hasDriven, turned;
	float setPoint = 60;
public:
	Robot() :
			//GYRO STUFF ONLY
			table(NULL),
			ahrs(NULL),
        	lw(NULL),
			autoLoopCounter(0),

			//DRIVING STUFF ONLY
			lDrive1(2) ,rDrive1(0) //left drive motors will be on 0 and 1
			,lDrive2(3), rDrive2(1) //right on 2 and 3
			,shifter(1, 0, 1)

			//SHOOTING STUFF ONLY
			,lowerFly(14) ,upperFly(13)
			,ballControl(12)
			,shooterLift(15)

			,intakeRoller(10)
			,intakeLift(4)
			,boulderIn(0)


			,driver(0)	//the joystick of the person in control of the drive train
			,operater(1) //the joystick of the person in control of everything else
{}
	void RobotInit(){
		//James Added SmartDashboard
		SmartDashboard::PutNumber("Dis",AutoSetDis);
		SmartDashboard::PutNumber("kP",kP);
		SmartDashboard::PutNumber("kI",kI);
		SmartDashboard::PutNumber("kD",kD);
		//prefs = Preferences::GetInstance();
		//AutoSetDis = prefs->GetDouble("AutoSetDis",0);
		//kP= prefs->GetDouble("kP",0);
		//kI= prefs->GetDouble("kI",0);
		//kD= prefs->GetDouble("kD",0);

		//Gyro stuff
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
        //i dont know what any of that stuff does, if you want to find out be my guest


        //Drive stuff
		lDrive2.SetControlMode(CANSpeedController::kFollower);	//so that they take input of other SRXs
		rDrive2.SetControlMode(CANSpeedController::kFollower);
		lDrive2.Set(2);	//sets it to the controller on port 0
		rDrive2.Set(0); //sets it to the controller on port 2
		lDrive1.SetFeedbackDevice(CANTalon::QuadEncoder);
		rDrive1.SetFeedbackDevice(CANTalon::QuadEncoder);
		//setting motor control modes

		lowerFly.SetFeedbackDevice(CANTalon::QuadEncoder);
		upperFly.SetFeedbackDevice(CANTalon::QuadEncoder);

		intakeLift.SetControlMode(CANTalon::kPosition);
		intakeLift.SetFeedbackDevice(CANTalon::QuadEncoder);
		intakeLift.SetPID(.5, 0, 12);
		shooterLift.SetControlMode(CANTalon::kPosition);
		shooterLift.SetFeedbackDevice(CANTalon::QuadEncoder);
		shooterLift.SetPID(.5, 0, 12);
	}
	/**
	 * Runs the motors with arcade steering.
	 */
	void Disabled(){
		while(IsDisabled())
		{
			if(driver.GetRawButton(1)){ autoChooser = 0; }
			if(driver.GetRawButton(2)){ autoChooser = 1; }
			if(driver.GetRawButton(3)){ autoChooser = 2; }
			if(driver.GetRawButton(4)){ autoChooser = 3; }
			if(autoChooser == 0){
				SmartDashboard::PutString("Chosen autonomous: ", "n0thing");
			}
			else if(autoChooser == 1){
				SmartDashboard::PutString("Chosen autonomous: ", "Drive forward Low bar");
			}
			else if(autoChooser == 2){
				SmartDashboard::PutString("Chosen autonomous: ", "Drive forward terrain");
			}
			else if(autoChooser == 3){
				SmartDashboard::PutString("Chosen autonomous: ", "Drive forward and shoot");
			}
			else {
				SmartDashboard::PutString("Chosen autonomous: ", "The heck you doin");
			}
			AutoSetDis = SmartDashboard::GetNumber("Dis",AutoSetDis);
			kP = SmartDashboard::GetNumber("kP",kP);
			kI = SmartDashboard::GetNumber("kI",kI);
			kD = SmartDashboard::GetNumber("kD",kD);
		}
	}

	void Autonomous(){
		lowerIntake = false;
//		intakeLift.SetPosition(40000);
//		if(autoChooser == 1){
			autoTime.Start();
//		}
		ahrs->ZeroYaw();
		intakeLift.SetPosition(36500); //47000 if no panel
		shifter.Set(DoubleSolenoid::kForward);
		//James Added
		lDrive1.SetEncPosition(0);
		rDrive1.SetEncPosition(0);
		//James Added End
		while(IsAutonomous() && IsEnabled()){
			SmartDashboard::PutBoolean("lowerintake", lowerIntake);
			if(!lowerIntake){
				if(autoChooser == 1){
				intakeLift.Set(0);
					if(intakeLift.GetPosition() < 300 && !lowerIntake){
						lowerIntake = true;
						autoTime.Reset();
					}
				}
				if(autoChooser == 2){
				intakeLift.Set(10000);
					if(intakeLift.GetPosition() < 7000 && !lowerIntake){
						lowerIntake = true;
						autoTime.Reset();
					}
				}
			}
			else{
				if((autoChooser == 1|| autoChooser == 2) && lowerIntake){ //if we are just driving forward with timing
					if(autoTime.Get() < 4.5){
						lDrive1.Set(-.5);
						rDrive1.Set(.5);
					}
					else{
					lDrive1.Set(0);
					rDrive1.Set(0);}
				}
			}
/*				else if(autoChooser == 2 || autoChooser == 3){
					curErr = ahrs->GetYaw();
					if(curErr > 300){
						curErr = curErr - 360;
					}
					curErr *= .2;
					avgDst = (lDrive1.GetPosition() + rDrive1.GetPosition())/2;
					if((avgDst - toDrive) < 0 && abs(avgDst - toDrive) < 100){
						lPow = (avgDst*.2 + .3) * (1 - curErr);
						rPow = -(avgDst*.2 + .3) * (1 + curErr);
					}
					else if ((avgDst - toDrive) > 0 && abs(avgDst - toDrive) < 100){
						lPow = -(avgDst*.2 + .3) * (1 + curErr);
						rPow = (avgDst*.2 + .3) * (1 - curErr);
					}
					if(abs(avgDst - toDrive) < 100){
						lPow = 0;
						rPow = 0;
						hasDriven = true;
					}
					lDrive1.Set(lPow);
					rDrive1.Set(rPow);
				}

				if(hasDriven && autoChooser == 3 && !turned){
					curErr = 60 - ahrs->GetYaw();
					if(abs(curErr) < -200){
						curErr = curErr + 360;
					}
					if(curErr > 0){
						lPow = curErr * .03 + .1;
						rPow = curErr * .03 + .1;
					}
					if(curErr < 1){
						lPow = 0; rPow = 0;
						turned = true;
					}
					lDrive1.Set(lPow);
					rDrive1.Set(rPow);
				}

				if(turned && autoChooser == 3){
					shooterLift.Set(160000);
					lowerFly.Set(1);
					upperFly.Set(1);
					if(shooterLift.GetClosedLoopError() < 500){
						ballControl.Set(-1);
					}
				}

			}
			*/
		}
		autoTime.Stop();

	}

	void OperatorControl()
	{
		lowerIntake = false;
	//	intakeLift.SetPosition(000);
		shooterLift.SetPosition(0);
		speedTime.Start();
		while (IsOperatorControl() && IsEnabled())
		{
			SmartDashboard::PutNumber("AutoSetDis", AutoSetDis);
			SmartDashboard::PutNumber("kP", kP);
			SmartDashboard::PutNumber("kI", kI);
			SmartDashboard::PutNumber("kD", kD);
			SmartDashboard::PutNumber("POV", operater.GetPOV());
			SmartDashboard::PutNumber("Left drive", lDrive1.GetPosition());
			SmartDashboard::PutNumber("Right drive", rDrive1.GetPosition());
			//GYRO STUFF
/*	        if ( !ahrs ) return;

	        bool reset_yaw_button_pressed = DriverStation::GetInstance().GetStickButton(0,1);
	        if ( reset_yaw_button_pressed ) {
	            ahrs->ZeroYaw();
	        }
*/
			//NAVX STUFF
			SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());
			SmartDashboard::PutNumber(  "IMU_Pitch",            ahrs->GetPitch());
			SmartDashboard::PutNumber(  "IMU_Roll",             ahrs->GetRoll());

			//DRIVING CODE--ONLY PERTAINS TO DRIVE TRAIN
			throttle = adjust(driver.GetRawAxis(1));	//uses function adjust in adjustValues.cpp
			turn = adjust(driver.GetRawAxis(4));	//it sets deadbands and rescales it
			lPow = leftPower(throttle, -turn);	//a fucntion for calculating drive power
			rPow = rightPower(throttle, -turn);	//it uses Omair's math, im not entirely sure what all it does

			lDrive1.Set(lPow);	//sets motor powers
			rDrive1.Set(rPow);

			if(driver.GetRawButton(4)){ shifter.Set(DoubleSolenoid::kForward); }
			else if(driver.GetRawButton(1)){ shifter.Set(DoubleSolenoid::kReverse); }

			//INTAKING
			if(operater.GetRawButton(2)){
				ballControlVal = 1;
				upperSpeed = -1;
				intakeRoller.Set(-1);
				if(!set){
					intakeVal = 4500;
					set = true;
				}
				shootIntake = true;
			}
			else if(operater.GetRawButton(4)){
				if(!shootIntake){
					speedTime.Reset();
					shootIntake = true;
				}
				if(!set){
					intakeVal = 4500;
					set = true;
				}
				upperSpeed = 1;
				intakeRoller.Set(1);
				if(speedTime.Get() > .5){
					ballControlVal = -1;
				}
			}
			else{ shootIntake = false; intakeRoller.Set(0);
			ballControlVal = 0; set = false;}

			if(shootIntake == false){
				upperSpeed = 0;;
			//	upperFly.Set(0);
			}

			if((intakeVal > 300 && adjust(operater.GetRawAxis(3)) > 0) || (intakeVal < 31000 && adjust(operater.GetRawAxis(3)) < 0))
			{
				intakeVal = intakeVal - adjust(operater.GetRawAxis(3)) * 200;
			}
			if(operater.GetRawButton(3)){
				position = 27000;
				intakeVal = 25000;
			}
			intakeLift.Set(intakeVal);

			//CONTROLLING FLYWHEELS
			if(operater.GetRawButton(6) || run){
				run = true;
				upperSpeed = 1;
			}
			if(operater.GetRawButton(5)){
				upperSpeed = 0;
				run = false;
			}
			SmartDashboard::PutBoolean("Run", run);
			//BALL CONTROLLER
			if((operater.GetRawButton(8)) && run){
				if(!shotReset){
					shotReset = true;
					speedTime.Reset();
				}

				if(speedTime.Get() > 1){
					run = false;
				}
				ballControlVal = -1;
			}
			else if(!operater.GetRawButton(8)){
				shotReset = false;
			}

			else if(operater.GetRawButton(6)){
				run = false;
			}
			//SHOOTER LIFT
			if(shooterLift.GetPosition() > 80000 || intakeLift.GetPosition() < 5000 || shooterLift.GetPosition() < 30000){
				if(operater.GetPOV() == 0){
					position = 175000;
				}
				else if(operater.GetPOV()==180){
					position = 2000;
					shootIntake = false;
					run = false;
				}
				if(position < shooterLift.GetPosition() && shooterLift.GetPosition() > 5000){
					shooterLift.SetPID(.1, 0, 30);
				}
				else{ shooterLift.SetPID(.5,0,30); }

				position = position - adjust(operater.GetRawAxis(1))*400;
			}

			ballControl.Set(ballControlVal);
			if(position < 0) { position = 0; }
			if(position > 200000){ position = 200000;}
			shooterLift.Set(position);

			lowerFly.Set(upperSpeed);
			upperFly.Set(upperSpeed);

	/*		if(operater.GetRawButton(12)){
				if(!lowerIntake){
					intakeLift.SetPosition(40000);
					intakeLift.Set(0);
					if(intakeLift.GetOutputCurrent() > 4){
						intakeLift.SetPosition(0);
						lowerIntake = true;
					}
				}
			}*/
		}

//		SmartDashboard::PutNumber("Upper flywheel speed", upperFly.GetEncVel());
//		SmartDashboard::PutNumber("Lower flywheel speed", lowerFly.GetEncVel());
			Wait(0.005);				// wait for a motor update time
	}

};

START_ROBOT_CLASS(Robot);
