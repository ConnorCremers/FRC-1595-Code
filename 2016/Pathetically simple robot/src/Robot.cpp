#include "WPILib.h"
#include "Functions.h"
#include "AHRS.h"
#include <iostream>
class Robot: public SampleRobot
{
	//GYRO STUFF--THE FOLLOWING IS FOR THE GYRO
	std::shared_ptr<NetworkTable> table;
    AHRS *ahrs;
    LiveWindow *lw;
    int autoLoopCounter;

	//DRIVE ITEMS--THE FOLLOWING ONLY HAS TO DO WITH DRIVE TRAIN ITEMS
	CANTalon lDrive1, rDrive1; //the controlled two motors on the drivetrain
	CANTalon lDrive2, rDrive2; //The two motors which will be set as followers
	float throttle, turn	//adjusted inputs from joysticks
	,lPow ,rPow;	//powers for drive train
	DoubleSolenoid shifter;

	//SHOOTER ITEMS--THE FOLLOWING ONLY HAS TO DO WITH THE SHOOTER
	CANTalon lowerFly, upperFly; //the two flywheels
	CANTalon ballControl; //thing to push ball to flywheels
	CANTalon shooterLift;
	float wheelSpeed; //how fast to move wheels
	bool raising; //does the person give the order to raise
	bool shootIntake;
	Timer speedTime;
	float lowerSpeed = 0, upperSpeed = 0;
	bool run;
	float position;


	//INTAKE ITEMS--THE FOLLOWING ONLY HAS TO DO WITH THE INTAKE
	CANTalon intakeRoller; //motor running the intake
	CANTalon intakeLift; //motor which raises/lowers intake
	float intakeVal; //position to set the lift

	//choosing what terrain it is crossing
	int obstacles[11] = {6,6,6,6,6,6,6,6,6,6,6};
	bool positioned, positionOnce; //whether or not the arm has been positioned
	bool autoDrive; //a boolean to store whether or not robot is operating autonomously while crossing terrain

	Joystick driver, operater, shotSpeed; //joysticks for driver and operator (i know i misspelled it, operator is a storage type)
	float ball;
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
			,shifter(1, 0,1)

			//SHOOTING STUFF ONLY
			,lowerFly(13) ,upperFly(14)
			,ballControl(12)
			,shooterLift(15)

			,intakeRoller(10)
			,intakeLift(4)


			,driver(0)	//the joystick of the person in control of the drive train
			,operater(1) //the joystick of the person in control of everything else
			,shotSpeed(2)
{}
	void RobotInit(){
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

		//setting motor control modes
	//	shooterLift.SetControlMode(CANSpeedController::kPosition);
	//	shooterLift.SetFeedbackDevice(CANTalon::QuadEncoder);
	//	shooterLift.SetPID(1.2, 0, 30);

	//	intakeLift.SetControlMode(CANSpeedController::kPosition);
	//	intakeLift.SetFeedbackDevice(CANTalon::QuadEncoder);
	//	intakeLift.SetPID(.8, 0, 10);
	}
	/**
	 * Runs the motors with arcade steering.
	 */
	void Disabled(){
		while(IsDisabled())
		{
		}
	}
	void OperatorControl()
	{
		speedTime.Start();
		while (IsOperatorControl() && IsEnabled())
		{
			//GYRO STUFF
	        if ( !ahrs ) return;

	        bool reset_yaw_button_pressed = DriverStation::GetInstance().GetStickButton(0,1);
	        if ( reset_yaw_button_pressed ) {
	            ahrs->ZeroYaw();
	        }

			//DRIVING CODE--ONLY PERTAINS TO DRIVE TRAIN
			throttle = adjust(driver.GetRawAxis(1));	//uses function adjust in adjustValues.cpp
			turn = adjust(driver.GetRawAxis(4));	//it sets deadbands and rescales it
			lPow = leftPower(throttle, -turn);	//a fucntion for calculating drive power
			rPow = rightPower(throttle, -turn);	//it uses Omair's math, im not entirely sure what all it does

			lDrive1.Set(lPow);	//sets motor powers
			rDrive1.Set(rPow);

			if(driver.GetRawButton(4)){
				shifter.Set(DoubleSolenoid::kForward);
			}
			else if(driver.GetRawButton(1)){
				shifter.Set(DoubleSolenoid::kReverse);
			}

			//INTAKING
			if(operater.GetRawButton(2)){
				ball = 1;
				lowerFly.Set(-1);
				upperFly.Set(-1);
				intakeRoller.Set(-1);
			}
			else if(operater.GetRawButton(4)){
				if(!shootIntake){
					speedTime.Reset();
					shootIntake = true;
				}
				lowerFly.Set(1);
				upperFly.Set(1);
				intakeRoller.Set(1);
				if(speedTime.Get() > 1){
					ball = -1;
				}
			}
			else{ shootIntake = false; intakeRoller.Set(0); ball = 0;}


		//	intakeVal = intakeVal + adjust(operater.GetRawAxis(3))*40;
		//	if(intakeLift.GetPosition() < 3000 && operater.GetRawAxis(3) < 0){
		//		intakeVal = 3000;
		//	}
			intakeVal = adjust(operater.GetRawAxis(3));
			intakeLift.Set(intakeVal);

			//CONTROLLING FLYWHEELS
			upperSpeed = adjust((shotSpeed.GetRawAxis(2))/2+.5);
			if(operater.GetRawButton(7) || run){
				run = true;
				lowerFly.Set(upperSpeed);
				upperFly.Set(upperSpeed);
			}

			//BALL CONTROLLER
			if((operater.GetRawButton(8))){
				ball = -1;
			}
			//SHOOTER LIFT
/*			if(operater.GetPOV() == 90){
				position = 160000;
			}
			else if(operater.GetPOV()==270){
				position = 0;
				run = false;
				lowerFly.Set(0);
				upperFly.Set(0);
			}
			if(shooterLift.GetClosedLoopError() < 500){
				raising = false;
			}
			if(position < shooterLift.GetPosition() && shooterLift.GetPosition() > 5000){
				shooterLift.SetPID(.1, 0, 30);
			}
			else{ shooterLift.SetPID(.5,0,30); }
			if(shooterLift.GetPosition() < 5000){
				shooterLift.SetPosition(0);
			}*/
			//position = position - adjust(operater.GetRawAxis(1))*400;
			position = adjust(operater.GetRawAxis(1));
			SmartDashboard::PutNumber("Lift position", position);
			SmartDashboard::PutNumber("Joystick output", adjust(operater.GetRawAxis(1)));
			ballControl.Set(ball);
			shooterLift.Set(position);
		}

			Wait(0.005);				// wait for a motor update time
	}

};

START_ROBOT_CLASS(Robot);
