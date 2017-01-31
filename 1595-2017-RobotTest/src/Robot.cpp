#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <CANTalon.h>
#include <Joystick.h>
#include <Solenoid.h>
#include <DoubleSolenoid.h>
#include "Functions.h"
using namespace frc;



class Robot: public IterativeRobot {
public:
	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		SmartDashboard::PutData("Auto Modes", &chooser);

		//Drive train
		lDrive2.SetControlMode(CANSpeedController::kFollower);	//so that they take input of other SRXs
		//lDrive3.SetControlMode(CANSpeedController::kFollower);
		rDrive2.SetControlMode(CANSpeedController::kFollower);	//so that they take input of other SRXs
		//rDrive3.SetControlMode(CANSpeedController::kFollower);
		lDrive2.Set(0);
		//lDrive3.Set(0);
		rDrive2.Set(2);
		//rDrive3.Set(2;)
		lDrive1.SetInverted(true);
		lDrive2.SetInverted(true);
		//lDrive3.SetInverted(true);

		//Gears
		if(dr.GetRawButton(1)){
			clamper.Set(DoubleSolenoid::kForward);
		}
		else if(dr.GetRawButton(2)){
			clamper.Set(DoubleSolenoid::kReverse);
		}
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
		turn = adjust(dr.GetRawAxis(4))/2;
		throttle = -adjust(dr.GetRawAxis(1));
		lPow = throttle + turn;
		rPow = throttle - turn;
		lDrive1.Set(lPow);
		rDrive1.Set(rPow);


		if(dr.GetRawButton(1)){
			clamper.Set(DoubleSolenoid::kForward);
		}
		if(dr.GetRawButton(2)){
					clamper.Set(DoubleSolenoid::kReverse);
				}
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	LiveWindow* lw = LiveWindow::GetInstance();
	SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

	//Controllers
	Joystick dr { 0 };

	//Drive train
	CANTalon lDrive1 { 0 };
	CANTalon lDrive2 { 1 };
	//CANTalon lDrive3 { 2 };
	CANTalon rDrive1 { 2 };
	CANTalon rDrive2 { 3 };
	//CANTalon rDrive3 { 5 };
	float throttle, turn;
	float lPow, rPow;

	//Gear
	DoubleSolenoid clamper { 1, 0, 1 };
};

START_ROBOT_CLASS(Robot)
