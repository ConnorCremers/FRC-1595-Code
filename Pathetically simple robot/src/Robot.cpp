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
	Solenoid outerLift; //the solenoid which lowers the outer wheels
	DoubleSolenoid shifter; //the solenoid which will shift the gearing
	bool manualLift; //for knowing if it can go back to auto lifting/lowering wheels
	float throttle, turn	//adjusted inputs from joysticks
	,lPow ,rPow;	//powers for drive train

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
	int position;


	//INTAKE ITEMS--THE FOLLOWING ONLY HAS TO DO WITH THE INTAKE
	CANTalon intakeRoller; //motor running the intake
	CANTalon intakeLift; //motor which raises/lowers intake
	int intakeVal; //position to set the lift
	DigitalInput boulderIn; //a switch to signal that the boulder is fully entered
	DigitalInput atBottom; //if the intake is at the bottom

	//choosing what terrain it is crossing
	int obstacles[11] = {6,6,6,6,6,6,6,6,6,6,6};
	bool positioned, positionOnce; //whether or not the arm has been positioned
	bool autoDrive; //a boolean to store whether or not robot is operating autonomously while crossing terrain

	Joystick driver, operater, shotSpeed; //joysticks for driver and operator (i know i misspelled it, operator is a storage type)

public:
	Robot() :
			//GYRO STUFF ONLY
			table(NULL),
			ahrs(NULL),
        	lw(NULL),
			autoLoopCounter(0),

			//DRIVING STUFF ONLY
			lDrive1(0) ,rDrive1(2) //left drive motors will be on 0 and 1
			,lDrive2(1), rDrive2(3) //right on 2 and 3
			,outerLift(0, 0) //solenoid contoller 0, port 0
			,shifter(0, 1, 6)

			//SHOOTING STUFF ONLY
			,lowerFly(4) ,upperFly(5)
			,ballControl(6)
			,shooterLift(7)

			,intakeRoller(8)
			,intakeLift(9)
			,boulderIn(4)
			,atBottom(5)


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
		lDrive2.Set(0);	//sets it to the controller on port 0
		rDrive2.Set(2); //sets it to the controller on port 2

		//setting motor control modes
		shooterLift.SetControlMode(CANSpeedController::kPosition);
		shooterLift.SetFeedbackDevice(CANTalon::QuadEncoder);
		shooterLift.SetPID(1.2, 0, 30);
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
			turn = adjust(driver.GetRawAxis(2));	//it sets deadbands and rescales it
			lPow = leftPower(throttle, turn);	//a fucntion for calculating drive power
			rPow = rightPower(throttle, turn);	//it uses Omair's math, im not entirely sure what all it does

			lDrive1.Set(lPow);	//sets motor powers
			rDrive1.Set(rPow);


			//INTAKING
			if(operater.GetRawButton(2)){
				ballControl.Set(1);
				lowerFly.Set(-.5);
				upperFly.Set(-.5);
				intakeRoller.Set(-1);
			}
			else if(operater.GetRawButton(4)){
				if(!shootIntake){
					speedTime.Reset();
					shootIntake = true;
				}
				lowerFly.Set(.5);
				upperFly.Set(.5);
				intakeRoller.Set(1);
				if(speedTime.Get() > 1){
					ballControl.Set(-1);
				}
			}
			else{ shootIntake = false; }


			intakeVal = adjust(operater.GetRawAxis(3));
			intakeLift.Set(intakeVal);

			//CONTROLLING FLYWHEELS
			upperSpeed = adjust((shotSpeed.GetRawAxis(2)/2)+.5);
			if(operater.GetRawButton(7) || run){
				run = true;
				lowerFly.Set(upperSpeed);
				upperFly.Set(upperSpeed);
			}

			//BALL CONTROLLER
			if((operater.GetRawButton(8))){
				ballControl.Set(-1);
			}

			//SHOOTER LIFT
			if(operater.GetPOV() == 90){
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
			}
			position = position - adjust(operater.GetRawAxis(1))*400;
			shooterLift.Set(position);
		}

			Wait(0.005);				// wait for a motor update time
	}

};

START_ROBOT_CLASS(Robot);
