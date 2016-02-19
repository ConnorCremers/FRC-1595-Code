#include "WPILib.h"
#include "Functions.h"
#include "AHRS.h"
#include <iostream>
class Robot: public SampleRobot
{
	//GYRO STUFF--THE FOLLOWING IS FOR THE GYRO
	std::shared_ptr<NetworkTable> table;
	Joystick stick; // only joystick
    AHRS *ahrs;
    LiveWindow *lw;
    int autoLoopCounter;

	//DRIVE ITEMS--THE FOLLOWING ONLY HAS TO DO WITH DRIVE TRAIN ITEMS
	CANTalon lDrive1, rDrive1; //the controlled two motors on the drivetrain
	CANTalon lDrive2, rDrive2; //The two motors which will be set as followers
	Solenoid outerLift; //the solenoid which lowers the outer wheels
	bool manualLift; //for knowing if it can go back to auto lifting/lowering wheels
	float throttle, turn	//adjusted inputs from joysticks
	,lPow ,rPow;	//powers for drive train
	//ENcoders-they changed declaration-i will look into it
	Encoder *lEnc, *rEnc;

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
<<<<<<< HEAD
	float potVal; //value of potentiometer
=======
	float potVal; //value of encoder, replace with proper declaration
>>>>>>> origin/master
	float intakeVal; //how fast to raise it
	bool operatorOverride; //does the operator override it (get button 5 doesn't mean much)
	bool getDaBall; //temporary bool bc get button 6 means nothing
	bool loseDaBall; //temporary bool bc get button 6 means nothing
	DigitalInput boulderIn; //a switch to signal that the boulder is fully entered
	DigitalInput atBottom; //if the intake is at the bottom


	//AUTO ITEMS--THE FOLLOW HAS TO DO WITH AUTO DRIVING AND THE WAY IT INTERACTS WITH THE DRIVE TRAIN
	//lining up with the thing
	bool hasLinedUp, toLineUp, lineDetected, hasTurned, setPointOnce;
	float lineAngle, setPoint, iErr, tErr, curVal, pastVal, curErr;

	//choosing what terrain it is crossing
	std::string obstacles[11] = { "n0thing", "terrain", "terrain", "terrain", "terrain", "terrain", "terrain", "terrain", "terrain", "terrain", "terrain" };
	//options are: gate, door, titer, low, draw
	bool hasSelected;
	bool autoDrive; //a boolean to store whether or not robot is operating autonomously while crossing terrain
	bool manualOverride; //a boolean to store if it is manually overridden (will just be a button in future)
	bool onTarget; //signal from jetson at end of goalfinding that target is correctly identified


	Joystick driver, operater; //joysticks for driver and operator (i know i misspelled it, operator is a storage type)

public:
	Robot() :
			//GYRO STUFF ONLY
			table(NULL),
			stick(0),		// as they are declared above.
			ahrs(NULL),
        	lw(NULL),
			autoLoopCounter(0),

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
	void RobotInit(){
		//Gyro stuff
        table = NetworkTable::GetTable("datatable");
        lw = LiveWindow::GetInstance();
        try {
            /* Communicate w/navX MXP via the MXP SPI Bus.                                       */
            /* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
            ahrs = new AHRS(SPI::Port::kMXP);
        } catch (std::exception ex ) {
            std::string err_string = "Error instantiating navX MXP:  ";
            err_string += ex.what();
            DriverStation::ReportError(err_string.c_str());
        }
        if ( ahrs ) {
            LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
        }

        //Drive stuff
		lDrive2.SetControlMode(CANSpeedController::kFollower);	//so that they take input of other SRXs
		rDrive2.SetControlMode(CANSpeedController::kFollower);
		lDrive2.Set(0);	//sets it to the controller on port 0
		rDrive2.Set(2); //sets it to the controller on port 2

		lEnc = new Encoder(0, 1, false, Encoder::EncodingType::k4X); //initializes encoders
		rEnc = new Encoder(2, 3, false, Encoder::EncodingType::k4X);
		lEnc->SetDistancePerPulse(.5);
		rEnc->SetDistancePerPulse(.5); //sets how far it goes with each tick (will calculate)

		//Getting terrain types from smart dashboard
		SmartDashboard::PutString("Obstacle at position one", obstacles[1]);
		SmartDashboard::PutString("Obstacle at position two", obstacles[2]);
		SmartDashboard::PutString("Obstacle at position three", obstacles[3]);
		SmartDashboard::PutString("Obstacle at position four", obstacles[4]);
		SmartDashboard::PutString("Obstacle at position five", obstacles[5]);
		SmartDashboard::PutString("Obstacle at position six", obstacles[6]);
		SmartDashboard::PutString("Obstacle at position seven", obstacles[7]);
		SmartDashboard::PutString("Obstacle at position eight", obstacles[8]);
		SmartDashboard::PutString("Obstacle at position nine", obstacles[9]);
		SmartDashboard::PutString("Obstacle at position ten", obstacles[10]);
	}
	/**
	 * Runs the motors with arcade steering.
	 */
	void Disabled(){
		while(IsDisabled())
		{
		obstacles[1] = SmartDashboard::GetString("Obstacle at position one", obstacles[1]);
		obstacles[2] = SmartDashboard::GetString("Obstacle at position two", obstacles[2]);
		obstacles[3] = SmartDashboard::GetString("Obstacle at position three", obstacles[3]);
		obstacles[4] = SmartDashboard::GetString("Obstacle at position four", obstacles[4]);
		obstacles[5] = SmartDashboard::GetString("Obstacle at position five", obstacles[5]);
		obstacles[6] = SmartDashboard::GetString("Obstacle at position six", obstacles[6]);
		obstacles[7] = SmartDashboard::GetString("Obstacle at position seven", obstacles[7]);
		obstacles[8] = SmartDashboard::GetString("Obstacle at position eight", obstacles[8]);
		obstacles[9] = SmartDashboard::GetString("Obstacle at position nine", obstacles[9]);
		obstacles[10] = SmartDashboard::GetString("Obstacle at position ten", obstacles[10]);
		}
	}
	void OperatorControl()
	{
		while (IsOperatorControl() && IsEnabled())
		{
			//GYRO STUFF
	        if ( !ahrs ) return;

	        bool reset_yaw_button_pressed = DriverStation::GetInstance().GetStickButton(0,1);
	        if ( reset_yaw_button_pressed ) {
	            ahrs->ZeroYaw();
	        }
	        //AUTO DRIVING-THIS IS ONLY FOR THINGS WITH AUTONOMOUS FUNCTION
	        //Lining up
			if(operater.GetRawButton(6) || toLineUp){ //if the operator says to line up or it has been lining up
				autoDrive = true; //robot is autonomously doing things
				toLineUp = true; //so it knows to keep on doing it
				if(!lineDetected && !hasTurned){ //if it doesn't see a line and hasn't turned
					lPow = -.35; //drive forward
					rPow = .35;
				}
				else if (lineAngle < 0 && !hasTurned){ //if the angle is less than zero and it hasn't turned
					pastVal = curVal;
					curVal = ahrs->GetYaw(); //set values
					if(!setPointOnce){ //if it hasn't defined the setpoint
						setPoint = ahrs->GetYaw() + lineAngle; //the setpoint is the current yaw plus the angle of the line
						if(setPoint > 180){ setPoint = setPoint - 360; } //if its too big wrap it around
						setPointOnce = true; //done setting angle
					}
					iErr = integrate(curVal, iErr, setPoint, .005, true); //get i error
					tErr = (pastVal, curVal, setPoint, .3, iErr, 0, true); //get total error
					lPow = tErr;
					rPow = 0; //set motors
					curErr = setPoint - ahrs->GetYaw;  //find the current error
					if(curErr > 180) { curErr -= 360; } //if its too big, make it smaller
					if(curErr < -180) { curErr += 360; } //if its too small, make it bigger
					if(fabs(curErr) < 5){ hasTurned = true; setPointOnce = false;} //if its within 5 degrees, done turning
				}
				else if(lineAngle > 0 && !hasTurned){ //functionally identical to prior loop
					pastVal = curVal;
					curVal = ahrs->GetYaw();
					if(!setPointOnce){
						setPoint = ahrs->GetYaw() + lineAngle;
						if(setPoint > 180){ setPoint = setPoint - 360; }
						setPointOnce = true;
					}
					iErr = integrate(curVal, iErr, setPoint, .005, true);
					tErr = (pastVal, curVal, setPoint, .3, iErr, 0, true);
					lPow = 0;
					rPow = tErr;
					curErr = setPoint - ahrs->GetYaw;
					if(curErr > 180) { curErr -= 360; }
					if(curErr < -180) { curErr += 360; }
					if(fabs(curErr) < 5){ hasTurned = true; setPointOnce = false;}
				}

				if(hasTurned && !lineDetected){ //if it has turned and sees no line
					lPow = .35; //drive backwards
					rPow = -.35;
				}
				if(hasTurned && lineDetected){ hasLinedUp = true; toLineUp = false; //if its turned and sees a line
				lineDetected = false; hasTurned = false;} //reset and pass along that it is lined up
			}

			//choosing what terrain crossing function to go for
			if(hasLinedUp){
				obstacles[0] = obstacles[operater.GetRawAxis(1)]; //have the operator select the terrain type
				if(obstacles[0] == "low"){
					//lower the arm or whatever
					if(!setPointOnce){
						setPoint = ahrs->GetYaw();
						lEnc->Reset();
						rEnc->Reset();
						setPointOnce = true;
					}
					while((lEnc->GetDistance() + rEnc->GetDistance())/2 < 120){ //if the avg is > 120 (will adjust)
						pastVal = curVal;
						curVal = ahrs->GetYaw();
						iErr = integrate(curVal, iErr, setPoint, .005, true); //get i error
						tErr = (pastVal, curVal, setPoint, .3, iErr, 0, true); //get total error
						lPow = -.4 * (1+tErr);
						rPow = .4 * (1-tErr);
					}
				}
			}


			//DRIVING CODE--ONLY PERTAINS TO DRIVE TRAIN
			if(!autoDrive || manualOverride){ //if not autonomously moving or manually overridden
				throttle = adjust(driver.GetRawAxis(1));	//uses function adjust in adjustValues.cpp
				turn = adjust(driver.GetRawAxis(4));	//it sets deadbands and rescales it
				lPow = leftPower(throttle, turn);	//a fucntion for calculating drive power
				rPow = rightPower(throttle, turn);	//it uses Omair's math, im not entirely sure what all it does
				hasLinedUp = false;
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
<<<<<<< HEAD
				if((getDaBall||loseDaBall) && !boulderIn.Get()){ //if we want to intake and we dont already have a ball
					if(getDaBall){ intakeRoller.Set(1); } //if we want to intake
					else{ intakeRoller.Set(-1); } //if we want to outtake
					intakeVal = (5 - potVal) * .25; //where 5 is the setpoint and .25 is the P value
					if(intakeVal > 0){intakeVal += .2;}  //add a bit so it moves when P doesn't do much
					else{intakeVal -= .2; }
					if(abs(5-potVal) > .2){ //if its close to desired value
						intakeLift.Set(intakeVal);
					}
=======
				if(getDaBall && !boulderIn.Get()){ //if we want to intake and we dont already have a ball
					intakeRoller.Set(1);  //get things rollin
					intakeVal = (5 - potVal) * .25; //where 5 is the setpoint and .25 is the P value
					if(intakeVal > 0){intakeVal += 1;}  //add a bit so it moves when P doesn't do much
					else{intakeVal -= 1; }
					intakeLift.Set(intakeVal);
>>>>>>> origin/master
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
