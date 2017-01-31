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
#include <SerialPort.h>
#include "AHRS.h"
#include "Functions.h"
using namespace frc;



class Robot: public IterativeRobot {
public:

    virtual void PIDWrite(double output) {
        this->rotateToAngleRate = output;
    }

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

	try {
        ahrs = new AHRS(SPI::Port::kMXP);
	}
     catch (std::exception& ex ) {
        std::string err_string = "Error instantiating navX MXP:  ";
        err_string += ex.what();
        DriverStation::ReportError(err_string.c_str());
    }
 //   turnController = new PIDController(kGP, kGI, kGD, kGF, ahrs, this);
 //   turnController->SetInputRange(-180.0f,  180.0f);
 //   turnController->SetOutputRange(-1.0, 1.0);
 //   turnController->SetAbsoluteTolerance(3);
 //   turnController->SetContinuous(true);
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
		rPi.Reset();
		rPi.Flush();
	}

	void TeleopPeriodic() {
		SmartDashboard::PutNumber("Recieved bytes", rPi.GetBytesReceived());
//			ahrs->ZeroYaw();
		if(dr.GetRawButton(1)){
			whichProg[0] = 1;
			rPi.Write(whichProg);
		}
		else{
			whichProg[0] = 2;
			rPi.Write(whichProg);
		}
			rPi.Read(values, rPi.GetBytesReceived());
		//	targetPixel = values[0] + values[1] + values[2];
			targetAngle = (targetPixel - midPixel) * pixelsToDegrees;
			SmartDashboard::PutNumber("TargetPixel", targetPixel);
			SmartDashboard::PutNumber("Values0", values[0]);
			SmartDashboard::PutNumber("Values1", values[1]);
			SmartDashboard::PutNumber("Values2", values[2]);
			SmartDashboard::PutNumber("Values3", values[3]);
			SmartDashboard::PutNumber("Values4", values[4]);
			SmartDashboard::PutNumber("Values5", values[5]);
			SmartDashboard::PutNumber("Yaw", ahrs->GetYaw());
			Wait(.05);
//		if(autoPlaceStage == 1 || autoPlaceStage == 3){
//			autoPlaceStage++;
//			turnController->Enable();
//			turnController->SetSetpoint(targetAngle);

//		}
/*		if(dr.GetRawButton(6)){ autoPlacing = true; autoPlaceFlush = false; autoPlaceStage = 1; }
		else if(dr.GetRawButton(5)){ autoPlacing = false; }

		if(!autoPlacing){
			turn = adjust(dr.GetRawAxis(4))/4;
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

		else{
			if(!autoPlaceFlush || rPi.GetBytesReceived() > 4){ rPi.Flush(); autoPlaceFlush = true; }
			if(rPi.GetBytesReceived() == 3){
				ahrs->ZeroYaw();
				rPi.Read(values, 3);
				targetPixel = values[0] + values[1];
				targetAngle = (targetPixel - midPixel) * pixelsToDegrees;
				Wait(100);
				if(autoPlaceStage == 1 || autoPlaceStage == 3){
					autoPlaceStage++;
					turnController->Enable();
					turnController->SetSetpoint(targetAngle);
				}
			}

			if(autoPlaceStage == 2){
				rotatePow = rotateToAngleRate;
				lDrive1.Set(rotateToAngleRate);
				rDrive1.Set(-rotateToAngleRate);
				if(fabs(ahrs->GetYaw()-targetAngle) < 3){
					autoPlaceStage = 3;
				}
			}

			if(autoPlaceStage == 4){
				if(fabs(targetAngle) < 3){ autoPlaceStage = 5; }
				else{ autoPlaceStage = 1; }
			}

			if(autoPlaceStage ==5){
				autoPlacing = false;
			}

		}
	*/}

	void TestPeriodic() {
        LiveWindow::GetInstance()->AddActuator("DriveSystem", "RotateController", turnController);
        if ( ahrs ) {
           LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
       }

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

	//Vision
	SerialPort rPi { 115200, SerialPort::kMXP, 8 , SerialPort::kParity_None, SerialPort::kStopBits_One }; //initializes serial port at 9600 baud

	bool autoPlacing;
	bool autoPlaceFlush;
	char values[6];
	int targetPixel, midPixel;
	float targetAngle, pixelsToDegrees = 3;
	int autoPlaceStage;
	char whichProg[1];
	//Gyro
    AHRS *ahrs;                         // navX MXP
    PIDController *turnController;
    double rotateToAngleRate;
    double rotatePow;
	double kGP = 0.03f;
	double kGI = 0.00f;
	double kGD = 0.00f;
	double kGF = 0.00f;
};

START_ROBOT_CLASS(Robot)
