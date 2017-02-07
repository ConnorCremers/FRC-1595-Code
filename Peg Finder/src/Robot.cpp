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
#include <Timer.h>
using namespace frc;



class Robot: public IterativeRobot, public PIDOutput {
public:

    virtual void PIDWrite(double output) {
        this->rotateToAngleRate = output;
    }

	void RobotInit() {
		SmartDashboard::PutNumber("P value", kGP);
		SmartDashboard::PutNumber("I value", kGI);
		SmartDashboard::PutNumber("D value", kGD);

		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		SmartDashboard::PutData("Auto Modes", &chooser);

		//Drive train
		lDrive1.SetFeedbackDevice(CANTalon::QuadEncoder); //declare the encoders to run them at times
		rDrive1.SetFeedbackDevice(CANTalon::QuadEncoder);
		lDrive1.ConfigEncoderCodesPerRev(1024);
		rDrive1.ConfigEncoderCodesPerRev(1024);

		lDrive2.SetControlMode(CANSpeedController::kFollower);	//so that they take input of other SRXs
		lDrive3.SetControlMode(CANSpeedController::kFollower);
		rDrive2.SetControlMode(CANSpeedController::kFollower);	//so that they take input of other SRXs
		rDrive3.SetControlMode(CANSpeedController::kFollower);
		lDrive2.Set(9); //lDrive1
		lDrive3.Set(9);
		rDrive2.Set(3); //rDrive1
		rDrive3.Set(3);
		rDrive1.SetInverted(true); //right drive train inverted
		rDrive2.SetInverted(true);
		rDrive3.SetInverted(true);

	try {
        ahrs = new AHRS(SPI::Port::kMXP);  //initialize Nav X on SPI bus of MXP port on roborio
	}
     catch (std::exception& ex ) {
        std::string err_string = "Error instantiating navX MXP:  "; //if something goes wrong with nav X
        err_string += ex.what();
        DriverStation::ReportError(err_string.c_str());
    }

    turnController = new PIDController(kGP, kGI, kGD, kGF, ahrs, this); //initialize and set up turning PID
    turnController->SetInputRange(-180.0f,  180.0f);
    turnController->SetOutputRange(-.75, .75);
    turnController->SetAbsoluteTolerance(3);
    turnController->SetContinuous(true);
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

	void Disabled(){
		while(IsDisabled()){
			kGP = SmartDashboard::GetNumber("P value", kGP);
			kGI = SmartDashboard::GetNumber("I value", kGI);
			kGD = SmartDashboard::GetNumber("D value", kGD);
		}
	}
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
		rotateTimer.Start();
		turnController->SetPID(kGP,kGI,kGD);

		lDrive1.SetControlMode(CANTalon::kSpeed);
		rDrive1.SetControlMode(CANTalon::kSpeed);
		curDrawTimer.Start();
	}

	void TeleopPeriodic() {
		SmartDashboard::PutNumber("Recieved bytes", rPi.GetBytesReceived());
		if(rPi.GetBytesReceived() == 3){
			rPi.Read(values,3);
		}

		/*if(dr.GetRawButton(1)){
			whichProg[0] = 1;
			rPi.Write(whichProg);
		}
		else if(dr.GetRawButton(2)){
			whichProg[0] = 2;
			rPi.Write(whichProg);
		}*/
//		if(autoPlaceStage == 1 || autoPlaceStage == 3){
//			autoPlaceStage++;
//			turnController->Enable();
//			turnController->SetSetpoint(targetAngle);

//		}
		if(dr.GetRawButton(6)){ autoPlacing = true; autoPlaceFlush = false; autoPlaceStage = 1; }
		else if(dr.GetRawButton(5)){ autoPlacing = false; }

		if(!autoPlacing){
			turn = adjust(dr.GetRawAxis(4))/3; //get the turn, scale it down
			throttle = -adjust(dr.GetRawAxis(1)); //get the throttle
			lPow = throttle - turn; //simple arcade drive
			rPow = throttle + turn;

			if((fabs(rPow) < .35 && fabs(lPow) < .35) || rDrive1.GetEncVel() * encOutputRatio < 340 || rDrive1.GetEncVel() * encOutputRatio < 340){
				//if both left and right have more power at low gear or (left or right) speeds are below 340 rpm
				rDrive1.SetPID(kLDP, kLDI, kLDD); //set low gear PIDs
				lDrive1.SetPID(kLDP, kLDI, kLDD);
				shifter.Set(DoubleSolenoid::kForward); //low gear
				highGear = false;
				rSpeed = rPow / .35 * 360; //map the values from -.35 to .35 -> from -360 to 360
				lSpeed = lPow / .35 * 360; //could this cause issues if rSpeed and lSpeed end up being over 360 in low gear?
			}
			else{
				if((rDrive1.GetOutputCurrent() < 80 && lDrive1.GetOutputCurrent() < 80) || (rDrive1.GetEncVel() * encOutputRatio < 800 && lDrive1.GetEncVel() * encOutputRatio < 800)){
					//if both aren't drawing too much current and we aren't going vroom vroom
					curDrawTimer.Reset();  //note it
				}


				if(curDrawTimer.Get() < 3){ //if both are okay in terms of current draw
					rDrive1.SetPID(kHDP, kHDI, kHDD); //high gear PIDS
					lDrive1.SetPID(kHDP, kHDI, kHDD);
					shifter.Set(DoubleSolenoid::kReverse); //high gear
					highGear = true;
					if(rPow > 0){ rSpeed = (rPow - .35) * 1080 + 360; } //map values to appropriate ranges
					else { rSpeed = (rPow + .35) * 1080 - 360; }

					if(lPow > 0){ lSpeed = (lPow - .35) * 1080 + 360; }
					else { lSpeed = (lPow + .35) * 1080 - 360; }
				}


				if(curDrawTimer.Get() > 3){
					//if we are drawing too much current
					shifter.Set(DoubleSolenoid::kForward); //low gear time
					highGear = false;
					rDrive1.SetPID(kLDP, kLDI, kLDD);
					lDrive1.SetPID(kLDP, kLDI, kLDD);
					rSpeed = 360; //just max dat speed. VRRROOOOOOMMMMMM
					lSpeed = 360;
				}
			}

			lDrive1.Set(lSpeed); //set speeds
			rDrive1.Set(rSpeed);

			if(dr.GetRawButton(1)){
				shifter.Set(DoubleSolenoid::kForward);
			}
			if(dr.GetRawButton(2)){
				shifter.Set(DoubleSolenoid::kReverse);
			}
		}

		else{
			shifter.Set(DoubleSolenoid::kForward);
			if(!autoPlaceFlush || rPi.GetBytesReceived() > 4){ rPi.Reset(); autoPlaceFlush = true; }

		//	if(rPi.GetBytesReceived() == 3){
				targetPixel = values[0] + values[1] + values[2];
				if(autoPlaceStage == 1 || autoPlaceStage == 3){
					ahrs->ZeroYaw();
					targetAngle = (targetPixel - midPixel) * pixelsToDegrees;
					autoPlaceStage++;
					turnController->Enable();
					turnController->SetSetpoint(targetAngle);
				}
			//}

			if(autoPlaceStage == 2){
				rotatePow = rotateToAngleRate;
				lDrive1.Set(rotateToAngleRate);
				rDrive1.Set(-rotateToAngleRate);
				if(fabs(ahrs->GetYaw()-targetAngle) > 3){
					rotateTimer.Reset();
				}
				if(rotateTimer.Get() > 1){
					autoPlaceStage = 3;
				}

			}

			if(autoPlaceStage == 4){
				if(fabs(targetAngle) < 3){ ahrs->ZeroYaw(); autoPlaceStage = 5; }
				else{ autoPlaceStage = 1; }
			}
			if(autoPlaceStage == 5){
				targetAngle = (targetPixel - midPixel) * pixelsToDegrees;
				turnController->SetSetpoint(targetAngle);
				rotatePow = rotateToAngleRate;
				lDrive1.Set(adjust(dr.GetRawAxis(1)) * (1-rotatePow/4));
				rDrive1.Set(adjust(dr.GetRawAxis(1)) * (1+rotatePow/4));
			}


		}
		SmartDashboard::PutNumber("Rotate time", rotateTimer.Get());
		SmartDashboard::PutNumber("Target angle", targetAngle);
		SmartDashboard::PutNumber("Auto place stage", autoPlaceStage);
		SmartDashboard::PutNumber("TargetPixel", targetPixel);

		SmartDashboard::PutNumber("Yaw", ahrs->GetYaw());
		SmartDashboard::PutBoolean("Autoplace", autoPlacing);

		SmartDashboard::PutNumber("P value", kGP);
		SmartDashboard::PutNumber("I value", kGI);
		SmartDashboard::PutNumber("D value", kGD);

		SmartDashboard::PutBoolean("High gear?", highGear);
	}

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
	CANTalon lDrive1 { 9 };
	CANTalon lDrive2 { 10 };
	CANTalon lDrive3 { 11 };
	CANTalon rDrive1 { 3 };
	CANTalon rDrive2 { 4 };
	CANTalon rDrive3 { 5 };
	DoubleSolenoid shifter { 1, 0, 1};
	float throttle, turn;
	float lPow, rPow, lSpeed, rSpeed;
	static float encOutputRatio = 5/3;
	float kLDP = .05, kLDI = 0, kLDD = 0;
	float kHDP = .05, kHDI = 0, kHDD = 0;
	bool highGear;
	Timer curDrawTimer;
	//Gear
	DoubleSolenoid clamper { 1, 2, 3 };

	//Vision
	SerialPort rPi { 115200, SerialPort::kMXP, 8 , SerialPort::kParity_None, SerialPort::kStopBits_One }; //initializes serial port at 9600 baud

	bool autoPlacing;
	bool autoPlaceFlush;
	Timer rotateTimer;
	char values[100];
	int targetPixel, midPixel = 320;
	float targetAngle, pixelsToDegrees = .0796875; //arctan(5.125/10.75)*2 / 640
	int autoPlaceStage;
	char whichProg[1];
	//Gyro
    AHRS *ahrs;                         // navX MXP
    PIDController *turnController;
    double rotateToAngleRate;
    double rotatePow;
	double kGP = 0.05;
	double kGI = 0.0005;
	double kGD = 0.0;
	double kGF = 0.00;
};

START_ROBOT_CLASS(Robot)
