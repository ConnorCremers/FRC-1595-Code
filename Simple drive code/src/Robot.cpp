#include "WPILib.h"
#include "Functions.h"
/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
class Robot: public SampleRobot
{
	CANTalon lDrive, rDrive; //the two motors on the drivetrain
	Joystick stick; // only joystick
	float throttle, turn	//adjusted inputs from joysticks
	,lPow ,rPow;	//powers for drive train
public:
	Robot() :
			lDrive(2) ,rDrive(3)
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
			throttle = adjust(stick.GetRawAxis(1));
			turn = adjust(stick.GetRawAxis(4));
			lPow = leftPower(throttle, turn);
			rPow = rightPower(throttle, turn);
			lDrive.Set(lPow);
			rDrive.Set(rPow);
			Wait(0.005);				// wait for a motor update time
		}
	}

};

START_ROBOT_CLASS(Robot);
