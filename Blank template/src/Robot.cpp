#include "WPILib.h"
/**
 * This sample shows how to use the new CANTalon to just run a motor in a basic
 *   throttle mode, in the same manner as you might control a traditional PWM
 *   controlled motor.
 *
 */
class Robot : public SampleRobot
{
	Joystick exampleJoystick; //joystick
	CANTalon demoMotor; //we use talons on our robot
	DigitalInput exampleSwitch; //just a digital input
	AnalogInput examplePotentiometer; //analog input
	Solenoid exampleSolenoid; //1 way solenoid
	DoubleSolenoid exampleDoubleSolenoid; //2 way solenoid

public:
	Robot() :
		exampleJoystick(0) //sets exampleJostick to position 0
		,demoMotor(0) //sets our CAN Talon to 0 on the CAN system
		,exampleSwitch(0) //sets switch to DIO 0
		,examplePotentiometer(0) //sets potentiometer to Analog in 0
		,exampleSolenoid(1, 0) //set solenoid to to solenoid controller 1, position 0
		,exampleDoubleSolenoid(1, 1, 2) //sets double solenoid to solenoid controller 1, position 1 for out, position 2 for in

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
		//put autonomous initialization stuff in here
		while (IsAutonomous() && IsEnabled()){
			//put code to be run while the robot is in autonomous
		}
	}

	void OperatorControl() {
		//put stuff to be done at the beginning of teleop
		while (IsOperatorControl() && IsEnabled()) {
			//put teleop stuff in here

			if(exampleJoystick.GetRawButton(1)){ //if a button is being pressed
				float speed = exampleJoystick.GetRawAxis(1); //set new variable "speed" to the value of axis 1
			}

			if(exampleSwitch.Get()){ //if the switch is returning true
				exampleSolenoid.Set(1); //has the example solenoid extend
				exampleSolenoid.Set(0); //example solenoid loses power
			}

			 //using GetVoltage() with analog sensors will give a value of 0-5
			//alternately, it can be GetValue() to return a value from 0 to 1024
			float motorPower = (examplePotentiometer.GetVoltage() - 2.5)/2.5; //creates variable motor power and places the voltage of potentiometer in it
			 //the reason for the -2.5 is so that its range is centered on zero (-2.5 to 2.5) so it can be used for a motor power
			//Dividing it by 2.5 brings it into the range of motors (-1 to 1 from -2.5 to 2.5)
			demoMotor.Set(motorPower); //sets the value of the potentiometer to the motor

			exampleDoubleSolenoid.Set(DoubleSolenoid::kForward); //sets the double solenoid to position out
			exampleDoubleSolenoid.Set(DoubleSolenoid::kReverse); //sets the double solenoid to position in


		}
	}


};

START_ROBOT_CLASS(Robot);
