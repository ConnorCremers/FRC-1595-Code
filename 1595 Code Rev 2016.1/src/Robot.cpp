#include "WPILib.h"
#include "AHRS.h"
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
	//GYRO STUFF--THE FOLLOWING IS FOR THE GYRO
	std::shared_ptr<NetworkTable> table;
    AHRS *ahrs;
    LiveWindow *lw;
    int autoLoopCounter;

	//DRIVE ITEMS--THE FOLLOWING ONLY HAS TO DO WITH DRIVE TRAIN ITEMS
	CANTalon lDrive1, rDrive1; //the controlled two motors on the drivetrain
	CANTalon lDrive2, rDrive2; //The two motors which will be set as followers
	DoubleSolenoid shifter; //the solenoid which will shift the gearing
	DoubleSolenoid PTO; //solenoid for PTO
	float throttle, turn	//adjusted inputs from joysticks
	,lPow ,rPow;	//powers for drive train

	//SHOOTER ITEMS--THE FOLLOWING ONLY HAS TO DO WITH THE SHOOTER
	CANTalon lowerFly, upperFly; //the two flywheels
	CANTalon ballControl; //thing to push ball to flywheels
	CANTalon shooterLift; //the lift for the shooter
	DigitalInput boulderIn; //the switch of whether or not ball is in
	DoubleSolenoid shooterBrake; //brake on the shooter lift

	//INTAKE ITEMS--THE FOLLOWING ONLY HAS TO DO WITH THE INTAKE
	CANTalon intakeRoller; //motor running the intake
	CANTalon intakeLift; //motor which raises/lowers intake

	Joystick dr, op; //joysticks for driver and operator (i know i misspelled it, operator is a storage type)

	double kPG, kIG, kDG, autoSetAngle;
	double kPD, kID, kDD, autoSetDis;
	double curAngle;
	Talon anglePID;
	PIDController *angleControl;

	float rotateToAngleRate;
public:
	Robot() :
		//GYRO STUFF ONLY
		table(NULL),
		ahrs(NULL),
    	lw(NULL),
		autoLoopCounter(0),

		//DRIVING STUFF ONLY
		lDrive1(2) ,rDrive1(0) //left drive motors will be on 0 and 1
		,lDrive2(3), rDrive2(1) //right on 2 and 3
		,shifter(1, 0, 1)
		,PTO(0, 0, 1)

		//SHOOTING STUFF ONLY
		,lowerFly(14) ,upperFly(13)
		,ballControl(12)
		,shooterLift(15)
		,boulderIn(0)
		,shooterBrake(0, 2, 3)

		,intakeRoller(10)
		,intakeLift(4)


		,dr(0)	//the joystick of the person in control of the drive train
		,op(1) //the joystick of the person in control of everything else
		,anglePID(0)
	{

	}

	void RobotInit(){
		//Gyro things
        table = NetworkTable::GetTable("datatable");
        lw = LiveWindow::GetInstance();
        try {
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
		lDrive2.Set(2);	//sets it to the controller on port 0
		rDrive2.Set(0); //sets it to the controller on port 2
		lDrive1.SetFeedbackDevice(CANTalon::QuadEncoder);
		rDrive1.SetFeedbackDevice(CANTalon::QuadEncoder);
		//setting motor control modes

		lowerFly.SetFeedbackDevice(CANTalon::QuadEncoder);
		upperFly.SetFeedbackDevice(CANTalon::QuadEncoder);

		intakeLift.SetControlMode(CANTalon::kPosition);
		intakeLift.SetFeedbackDevice(CANTalon::QuadEncoder);
		intakeLift.SetPID(.5, 0, 12);
		shooterLift.SetControlMode(CANTalon::kPosition);
		shooterLift.SetFeedbackDevice(CANTalon::AnalogPot);
		shooterLift.SetPID(.5, 0, 12);

		SmartDashboard::PutNumber("kPG", kPG);
		SmartDashboard::PutNumber("kIG", kIG);
		SmartDashboard::PutNumber("kDG", kDG);
		SmartDashboard::PutNumber("kPD", kPD);
		SmartDashboard::PutNumber("kID", kID);
		SmartDashboard::PutNumber("kDD", kDD);
		SmartDashboard::PutNumber("autoSetDis", autoSetDis);
		SmartDashboard::PutNumber("autoSetAngle", autoSetAngle);
		angleControl = new PIDController(kPG, kIG, kDG, ahrs, &lDrive1);
		angleControl->SetContinuous(true);
		angleControl->SetInputRange(-180, 180);
	}

	void Disabled(){
		while(IsDisabled()){
			kPG = SmartDashboard::GetNumber("kPG", kPG);
			kIG = SmartDashboard::GetNumber("kIG", kIG);
			kDG = SmartDashboard::GetNumber("kDG", kDG);
			kPD = SmartDashboard::GetNumber("kPD", kPD);
			kID = SmartDashboard::GetNumber("kID", kID);
			kDD = SmartDashboard::GetNumber("kDD", kDD);
			autoSetDis = SmartDashboard::GetNumber("autoSetDis", autoSetDis);
			autoSetAngle = SmartDashboard::GetNumber("autoSetAngle", autoSetAngle);
		}
	}

	void Autonomous(){
		angleControl->SetSetpoint(179.9);
		ahrs->ZeroYaw();
		angleControl->Enable();
		while (IsAutonomous() && IsEnabled()){
			SmartDashboard::PutNumber("kPG", kPG);
			SmartDashboard::PutNumber("kIG", kIG);
			SmartDashboard::PutNumber("kDG", kDG);
			SmartDashboard::PutNumber("kPD", kPD);
			SmartDashboard::PutNumber("kID", kID);
			SmartDashboard::PutNumber("kDD", kDD);
	        SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());
	        SmartDashboard::PutNumber(  "IMU_Pitch",            ahrs->GetPitch());
	        SmartDashboard::PutNumber(  "IMU_Roll",             ahrs->GetRoll());
			SmartDashboard::PutNumber("autoSetDis", autoSetDis);
			SmartDashboard::PutNumber("autoSetAngle", autoSetAngle);
			SmartDashboard::PutNumber("PID output", rotateToAngleRate);
			SmartDashboard::PutNumber("Talon value", anglePID.Get());


		}
	}

	 void PIDWrite(float output) {
				rotateToAngleRate = output;
				 }

	void OperatorControl()
	{
		while (IsOperatorControl() && IsEnabled())
		{

		}
	}

};

START_ROBOT_CLASS(Robot)
