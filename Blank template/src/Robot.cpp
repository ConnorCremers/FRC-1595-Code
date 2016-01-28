#include "WPILib.h"
/**
 * This sample shows how to use the new CANTalon to just run a motor in a basic
 *   throttle mode, in the same manner as you might control a traditional PWM
 *   controlled motor.
 *
 */
class Robot : public SampleRobot
{
	Joystick exampleJoystick;
	CANTalon demoMotor;
	DigitalInput exampleSwitch;
	AnalogInput examplePotentiometer;
	Solenoid exampleSolenoid;
	DoubleSolenoid exampleDoubleSolenoid;
	Encoder exampleEncoder = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
public:
	Robot() :
		exampleJoystick(0)
		,demoMotor(0)
		,exampleSwitch(0)
		,examplePotentiometer(0)
		,exampleSolenoid(1, 0)
		,exampleDoubleSolenoid(1, 1, 2)

{}


// [CDT GCC Built-in Compiler Settings] options
 void RobotInit(){
	 //what to do when the robot first starts up (initialize sensors)
 }
 void Disabled() {
  	while (IsDisabled()) {
  		//put stuff to do while disabled (ex: choose an auto to run)
  	}
  }

	void Autonomous(void){
		//put autonomous stuff in here
	}

	void OperatorControl() {
		while (IsOperatorControl() && IsEnabled()) {
			//put teleop stuff in here
		}
	}


};

START_ROBOT_CLASS(Robot);
