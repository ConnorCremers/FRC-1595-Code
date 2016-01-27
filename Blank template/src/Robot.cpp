#include "WPILib.h"
/**
 * This sample shows how to use the new CANTalon to just run a motor in a basic
 *   throttle mode, in the same manner as you might control a traditional PWM
 *   controlled motor.
 *
 */
class Robot : public SampleRobot
{
	float number;//need this so it doesnt have an error. Makes sense, right?
public:
	Robot() :
		number(3)
{}



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
