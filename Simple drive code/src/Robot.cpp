#include "WPILib.h"
#include "Functions.h"

class Robot: public SampleRobot
{
	CANTalon lDrive, rDrive; //the two motors on the drivetrain
	Joystick stick; // only joystick
	float throttle, turn	//adjusted inputs from joysticks
	,lPow ,rPow;	//powers for drive train
public:
	Robot() :
			lDrive(2) ,rDrive(3) //declaring the ports of the motors/joystick
			,stick(0)
	{}
	void robotinit(){

	}
	/**
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl()
	{
		while (IsOperatorControl() && IsEnabled())
		{
			throttle = adjust(stick.GetRawAxis(1));	//uses function adjust in adjustValues.cpp
			turn = adjust(stick.GetRawAxis(4));	//it sets deadbands and rescales it
			lPow = leftPower(throttle, turn);	//a fucntion for calculating drive power
			rPow = rightPower(throttle, turn);	//it uses Omair's math, im not entirely sure what all it does
			lDrive.Set(lPow);	//sets motor powers
			rDrive.Set(rPow);
			Wait(0.005);				// wait for a motor update time
		}
	}

};

START_ROBOT_CLASS(Robot);
