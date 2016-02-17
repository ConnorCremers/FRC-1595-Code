#include "WPILib.h"
#include "Functions.h"

class Robot: public SampleRobot
{
	//DRIVE ITEMS--THE FOLLOWING ONLY HAS TO DO WITH DRIVE TRAIN ITEMS
	CANTalon lDrive1, rDrive1; //the controlled two motors on the drivetrain
	CANTalon lDrive2, rDrive2; //The two motors which will be set as followers
	Solenoid outerLift; //the solenoid which lowers the outer wheels
	bool manualLift; //for knowing if it can go back to auto lifting/lowering wheels
	float throttle, turn	//adjusted inputs from joysticks
	,lPow ,rPow;	//powers for drive train
	//ENcoders-they changed declaration-i will look into it

	//SHOOTER ITEMS--THE FOLLOWING ONLY HAS TO DO WITH THE SHOOTER
	CANTalon lowerFly, upperFly; //the two flywheels
	CANTalon ballControl; //thing to push ball to flywheels
	float wheelSpeed; //how fast to move wheels
	float encoderRPMS; //placeholder till i figure out encoders
	//Encoders will be placed on both lower and upper fly
	bool raising; //does the person give the order to raise
	float distance; //the distance between the shooter and the target to determine speed
	bool autoShoot, manualShoot; //booleans for driver actions because "get button 3" means very little

	//INTAKE ITEMS--THE FOLLOWING ONLY HAS TO DO WITH THE INTAKE
	CANTalon intakeRoller; //motor running the intake
	CANTalon intakeLift; //motor which raises/lowers intake
	//Encoder to detect position of the intake
	float potVal; //value of encoder, replace with proper declaration
	float intakeVal; //how fast to raise it
	bool operatorOverride; //does the operator override it (get button 5 doesn't mean much)
	bool getDaBall; //temporary bool bc get button 6 means nothing
	DigitalInput boulderIn; //a switch to signal that the boulder is fully entered
	DigitalInput atBottom; //if the intake is at the bottom


	//AUTO ITEMS--THE FOLLOW HAS TO DO WITH AUTO DRIVING AND THE WAY IT INTERACTS WITH THE DRIVE TRAIN
	bool autoDrive; //a boolean to store whether or not robot is operating autonomously while crossing terrain
	bool manualOverride; //a boolean to store if it is manually overridden (will just be a button in future)
	bool onTarget; //signal from jetson at end of goalfinding that target is correctly identified

	Joystick driver, operater; //joysticks for driver and operator (i know i misspelled it, operator is a storage type)

public:
	Robot() :
			//DRIVING STUFF ONLY
			lDrive1(0) ,rDrive1(2) //left drive motors will be on 0 and 1
			,lDrive2(1), rDrive2(3) //right on 2 and 3
			,outerLift(0, 0) //solenoid contoller 0, port 0

			//SHOOTING STUFF ONLY
			,lowerFly(4) ,upperFly(5)
			,ballControl(6)

			,intakeRoller(7)
			,intakeLift(8)
			,boulderIn(0)
			,atBottom(1)


			,driver(0)	//the joystick of the person in control of the drive train
			,operater(1) //the joystick of the person in control of everything else
	{}
	void robotinit(){
		lDrive2.SetControlMode(CANSpeedController::kFollower);	//so that they take input of other SRXs
		rDrive2.SetControlMode(CANSpeedController::kFollower);
		lDrive2.Set(0);	//sets it to the controller on port 0
		rDrive2.Set(2); //sets it to the controller on port 2
	}
	/**
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl()
	{
		while (IsOperatorControl() && IsEnabled())
		{
			//DRIVING CODE--ONLY PERTAINS TO DRIVE TRAIN
			if(!autoDrive || manualOverride){ //if not autonomously moving or manually overridden
				throttle = adjust(driver.GetRawAxis(1));	//uses function adjust in adjustValues.cpp
				turn = adjust(driver.GetRawAxis(4));	//it sets deadbands and rescales it
				lPow = leftPower(throttle, turn);	//a fucntion for calculating drive power
				rPow = rightPower(throttle, turn);	//it uses Omair's math, im not entirely sure what all it does
			}
			else {
				//Place code for the automatic driving here (Case structure?)
			}


			if(driver.GetRawButton(6)){ //if driver wants them lowered
				outerLift.Set(1); //lower the outer wheels
				manualLift = true; //program doesn't get to do what it likes
			}
			else if(driver.GetRawButton(5)){
				outerLift.Set(0); //if driver hits a button, lift
				manualLift = true;
			}
			else { manualLift = false; } //tell program it can do what it likes-will be defined in auto driving code


			lDrive1.Set(lPow);	//sets motor powers
			rDrive1.Set(rPow);


			//SHOOTING CODE ONLY
			if(manualShoot){ //if operator is forcing shot
				ballControl.Set(-1); //shoot
			}
			else if(onTarget){ //if it is centered on target start running flywheels
				wheelSpeed = distance*5; //whatever it is to convert distance to wheel speed
				lowerFly.Set(wheelSpeed * .75); //set motors (value should be in rpms-we will figure that out)
				upperFly.Set(wheelSpeed); //i think upper one should be going faster, we'll see
				if(abs(encoderRPMS/wheelSpeed -1) < .05 && autoShoot){ //if it is close enough to desired speed and operator wants to shoot
					ballControl.Set(-1); //shoot
					Wait(1); //small delay to ensure successful shot
				}
			}

			//INTAKING STUFF ONLY
			if(!autoDrive || operatorOverride){//if the computer isn't in control of things for whatever
				if(getDaBall && !boulderIn.Get()){ //if we want to intake and we dont already have a ball
					intakeRoller.Set(1);  //get things rollin
					intakeVal = (5 - potVal) * .25; //where 5 is the setpoint and .25 is the P value
					if(intakeVal > 0){intakeVal += 1;}  //add a bit so it moves when P doesn't do much
					else{intakeVal -= 1; }
					intakeLift.Set(intakeVal);
				}
				else if(raising && !atBottom.Get()){ //if the shooter is told to be raised up
					intakeLift.Set(-1); //go down
				}
				else{
					intakeLift.Set(0); //don't move
				}
			}

			Wait(0.005);				// wait for a motor update time
		}
	}

};

START_ROBOT_CLASS(Robot);
