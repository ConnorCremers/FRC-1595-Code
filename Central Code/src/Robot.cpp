#include "Functions.h"
#include "WPILib.h"
#include "Timer.h"
#include "IMU.h"
#include "IMUAdvanced.h"
#include "Math.h"
#include "String.h"
/**
 * This sample shows how to use the new CANTalon to just run a motor in a basic
 *   throttle mode, in the same manner as you might control a traditional PWM
 *   controlled motor.
 *
 */
class Robot : public SampleRobot {
	Joystick Mikey, James;				//the joysticks where Mikey is totes/containers and James is drive train

			//The following are all items for controlling the containers
			CANTalon contLift;					//lift for the containers
			DoubleSolenoid contGripper;				//grips containers(on = grip, off = open by spring)
			DoubleSolenoid liftLock;			//locks the lift to prevent backdrive
			//DigitalInput botLimit ,topLimit
			//,contPresent;	//limit switches on top and bottom of lift to ensure nothing breaks
			AnalogInput pot;					//where the lift is
			float liftPow;						//to receive input from function and run motor
			//int MikeyHit;						//the height to go to
			//bool liftManual	= true		//to go to top and bottom
			//,gripperCycle ,loopOne		//some various booleans
			//,upOrDown;					//changes from open to close
			//bool gripSet = false;


			//The following are for the drive train
			CANTalon lDrive1 ,lDrive2 ,rDrive1 ,rDrive2;			//controllers for the drive train
			float lPow ,rPow
			,throttle , turn
			,modifier = .6, lastModifier = .6;						//control for hte drive train
			Encoder *right = new Encoder(2,3,false,Encoder::EncodingType::k4X);
			Encoder *left = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
			float pastThrottle, pastTurn;
			NetworkTable *table;				//all the fancy stuff for the Nav X
			//int loopCoJamest;
			float past, current;
			float iE, tE;
			const double pC = .035, iC = .02, dC = 0;
			IMU *imu;
			SerialPort *serial_port;
			bool first_iteration;	//end nav X
			//float wait = .1;
			//The follow are for totes
			Timer toteTime, toteTime2;;
			DoubleSolenoid toteLift, toteLiftL;
			DoubleSolenoid intakeOut;
			DigitalInput toteDetector		//The photoswitch
			,liftDown;						//lift is at bottom
			int liftSet ,liftPosition;		//used for controlling the lift
			CANTalon intakeMotor, intakeMotor2;			//conveyor belt
			bool toteManual,toteLoopOne, otherLoopOne;
			int autoToDo = 1;
			std::string autoOut;
			Timer autoTimer;
			bool triggered, goneDown;//, liftCont, atBottom;
			float intakeVal;

			DoubleSolenoid coOp, contHook;
			DigitalInput leftHook, rightHook;

			float angleTarget,angleError,heading,headingAdjust;
			float encoderError, encoderTarget;
			int autoStage =0;
			double Rotate;
			float autoDis1=0,autoDis2=48,autoDis3=150,autoDis4=60;
			float autoTurn1=180,autoTurn2=180,autoTurn3=90;
			double autoHeightTime1=0.7,autoHeightTime2=1;
			Timer contTimer,whatever;

			bool check1,check2,check3,check4;
public:
 Robot()
     : Mikey(1) ,James(0)

	,contLift(0)
	,contGripper(0 ,0,1) ,liftLock(0 ,6,7)
	//	,botLimit(9) ,topLimit(8) ,contPresent(10)
	,pot(0)

	,lDrive1(6) ,lDrive2(7),rDrive1(4), rDrive2(5)


	,toteLift(1 ,4, 5)
	,toteLiftL(1,2,3)
	,intakeOut(0 ,4,5)
	,toteDetector(9)
	,liftDown(6)
	,intakeMotor(2), intakeMotor2(3)
	,coOp(1, 0, 1)
	,contHook(0, 2, 3)
	,leftHook(5), rightHook(4)
 {}

	/**
	 * Runs the motor from the output of a Joystick.
	 */

 void RobotInit(){
	 toteTime.Start();
	 rDrive2.SetControlMode(CANSpeedController:: kFollower);	//sets the second motor to the first
	 rDrive2.Set(4);
	 lDrive2.SetControlMode(CANSpeedController:: kFollower);
	 lDrive2.Set(6);

	 left->SetDistancePerPulse(M_PI/144);
	 right->SetDistancePerPulse(M_PI/144);

	 toteManual = false;
	 //some more fancy stuff for Nav X
	 table = NetworkTable::GetTable("datatable");
	 serial_port = new SerialPort(57600,SerialPort::kMXP);
	 uint8_t update_rate_hz = 50;
	 imu = new IMU(serial_port,update_rate_hz);
	 if ( imu ) {
	 	LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", imu);
	 }
	 first_iteration = true;
 }
 void Disabled() {
  	while (IsDisabled()) {
 	if(Mikey.GetRawButton(3)) {autoToDo = 0;
 	autoOut = "Nothing.";}
	if(Mikey.GetRawButton(4)) {autoToDo = 1;
	autoOut = "3 Bin Stack.";}
 	if(Mikey.GetRawButton(5)) { autoToDo= 2;
 	autoOut = "Snatch and run.";}
 	if(Mikey.GetRawButton(2)){ autoToDo = 3;
 		autoOut = "Bins from step. ";	}
 	SmartDashboard::PutString("What to do in auto: ", autoOut);
	SmartDashboard::PutBoolean("Left Step Grabber", leftHook.Get());
	SmartDashboard::PutBoolean("Right Step Grabber", rightHook.Get());
	toteTime2.Start();
  	}
  }

	void Autonomous(void){
 		autoTimer.Start();
 		autoTimer.Reset();
  		headingAdjust = -90 - imu->GetYaw();
  		 autoStage = 0;
  		check1=false;
  		check2=false;
  		check3=false;
  		check4=false;
  		left->Reset();
  		imu->ZeroYaw();
  		Wait(.3);
	while(IsAutonomous() && IsEnabled()){
	  if(autoToDo == 1){
		  heading=imu->GetYaw()+headingAdjust;
			if(heading<-180)
			{
				heading=heading+360;
			}
			if(heading>180)
			{
				heading=heading-360;
			}


			if(autoStage == 0)
			{
			left->Reset();
			//rEncoder->Reset();
			angleTarget=heading;
			autoStage=1;
			//Start contLift
			contGripper.Set(DoubleSolenoid::kReverse);                 //walawala
			//contLift.Set(-0.8);										   //Speed
			//liftLock.Set(DoubleSolenoid::kReverse);					//Lock Direction
			//Start contLift
			//Start Intake
			intakeMotor.Set(-0.8);            						//Motor Direction
			intakeMotor2.Set(0.8);
			//Start Intake
			whatever.Start();
			whatever.Reset();
			contTimer.Start();
			intakeOut.Set(DoubleSolenoid::kForward);
			}



			if(autoStage==1)
			{
				if(whatever.Get()>0.3 && whatever.Get()<autoHeightTime1)
				{
					contLift.Set(-0.8);										   //Speed
					liftLock.Set(DoubleSolenoid::kReverse);
				}
				if(whatever.Get()>autoHeightTime1)
				{
					contLift.Set(0);
					liftLock.Set(DoubleSolenoid::kForward);
					check4=true;
					//check1=true;
				}

				if(toteDetector.Get()==false)						//Tote detect
				{
					toteLift.Set(DoubleSolenoid::kForward);
					toteLiftL.Set(DoubleSolenoid::kForward);
					intakeMotor.Set(-0.2);
					intakeMotor2.Set(0.2);
					if(check1==false)
					{
						//contTimer.Reset();
						check1=true;
					}
					if(liftDown.Get()==false)							//Can be improved
					{
						check3=true;
					}
				}

				encoderError= autoDis1-left->GetDistance();
				if(abs(encoderError) < 2)
				{
					throttle = 0;
				}
				else
				{
					throttle = 0.25;
				}
				angleError= angleTarget-heading;
				lPow = throttle + angleError*0.015;					//getting motor values
				rPow = throttle - angleError*0.015;
				lDrive1.Set(lPow);
				//lDrive2.Set(lPow);
				rDrive1.Set(-rPow);
				//rDrive2.Set(-rPow)
				if(fabs(encoderError)<3)
				{
					lDrive1.Set(0);
					//lDrive2.Set(0);
					rDrive1.Set(0);
					//rDrive2.Set(0);
					check2=true;
				}
				if(check1==true && check2 ==true && check3==true && check4 ==true)
				{
					toteLift.Set(DoubleSolenoid::kReverse);
					intakeOut.Set(DoubleSolenoid::kReverse);
					toteLiftL.Set(DoubleSolenoid::kReverse);
					autoStage=2;
					angleTarget=90;
					check1=false;
					check2=false;
					check3=false;
					check4=false;
				}
			}

			if(autoStage==2)
			{
				angleError= angleTarget-heading;

				lPow = angleError*0.018;					//getting motor values
				rPow = -angleError*0.018;
				if(lPow>0.4)
						{
							lPow=0.4;
							rPow=-0.4;
						}
				if(lPow<-0.4)
				{
					lPow=-0.4;
					rPow=0.4;
				}
				lDrive1.Set(lPow);
				//lDrive2.Set(lPow);
				rDrive1.Set(-rPow);
				//rDrive2.Set(-rPow);
				if(fabs(angleError<8))
				{
					//Wait(0.5);
					lDrive1.Set(0);
					//lDrive2.Set(0);
					rDrive1.Set(0);
					//rDrive2.Set(0);
					autoStage=3;
					left->Reset();
					//rEncoder->Reset();
					//Other
					//contTimer.Reset();
					//
					intakeMotor.Set(-0.8);
					intakeMotor2.Set(0.8);
					//
					//contLift.Set(0.8);
					//liftLock.Set(DoubleSolenoid::kReverse);
					whatever.Reset();
				}
			}

			if(autoStage==3)
			{
				//if(contTimer.Get()>autoHeightTime2)
				//{
					//contLift.Set(0);
					//liftLock.Set(DoubleSolenoid::kForward);
					//check1=true;
				//}


				if(toteDetector.Get()==false && whatever.Get()>0.5 && check1==false)
				{
					toteLift.Set(DoubleSolenoid::kForward);
					toteLiftL.Set(DoubleSolenoid::kForward);
					intakeMotor.Set(-0.2);
					intakeMotor2.Set(0.2);
						//contTimer.Reset();
						check1=true;
					//if(contTimer.Get()>1)							//Lift switch
					//{
						//check3=true;
					//}
				}

				if(liftDown.Get()==false && check1==true && check4==false && check3==true)
				{
					toteLift.Set(DoubleSolenoid::kReverse);
					toteLiftL.Set(DoubleSolenoid::kReverse);
					intakeOut.Set(DoubleSolenoid::kReverse);
					check4=true;
				}

				encoderError= 28-left->GetDistance();
				angleError= angleTarget-heading;
				if(abs(encoderError) < 8 && check3==false)
				{
					intakeOut.Set(DoubleSolenoid::kForward);
					check3= true;

				}
				if(abs(encoderError) < 5)
				{
					throttle = encoderError*0.02;
				}
				else
				{
					throttle = 0.3;
				}
				lPow = throttle + angleError*0.015;					//getting motor values
				rPow = throttle - angleError*0.015;
				lDrive1.Set(lPow);
				//lDrive2.Set(lPow);
				rDrive1.Set(-rPow);
				//rDrive2.Set(-rPow);
				if(fabs(encoderError)<2)
				{
					lDrive1.Set(0);
					//lDrive2.Set(0);
					rDrive1.Set(0);
					//rDrive2.Set(0);
					check2=true;
				}

				if(check1==true && check2 ==true && check4 ==true)
				{
					//toteLift.Set(DoubleSolenoid::kReverse);
					//toteLift2.Set(DoubleSolenoid::kReverse);
					autoStage=4;
					angleTarget=-65;//-110
					check1=false;
					check2=false;
					check3=false;
					check4=false;
				}
			}

			if(autoStage==4)
			{
			//	if(liftDown.Get()==false)
			//	{
			//		toteLift.Set(DoubleSolenoid::kReverse);
			//		intakeOut.Set(DoubleSolenoid::kReverse);
			//	}

				angleError= angleTarget-heading;
				lPow = angleError*0.018;					//getting motor values
				rPow = -angleError*0.018;
				if(lPow>0.5)
						{
							lPow=0.5;
							rPow=-0.5;
						}
				if(lPow<-0.5)
				{
					lPow=-0.5;
					rPow=0.5;
				}
				lDrive1.Set(lPow);
				//lDrive2.Set(lPow);
				rDrive1.Set(-rPow);
				//rDrive2.Set(-rPow);
				if(fabs(angleError)<8)
				{
					lDrive1.Set(0);
					//lDrive2.Set(0);
					rDrive1.Set(0);
					//rDrive2.Set(0);
					autoStage=5;
					left->Reset();
					//rEncoder->Reset();
					intakeMotor.Set(-0.8);
					intakeMotor2.Set(0.8);
				//	toteLift.Set(DoubleSolenoid::kForward);
					contTimer.Reset();
				}
			}
			if(autoStage==5)
			{
				//if(liftDown.Get()==false)
			//	{
			//		toteLift.Set(DoubleSolenoid::kReverse);
			//		intakeOut.Set(DoubleSolenoid::kReverse);
			//		intakeMotor.Set(-0.8);
			//		intakeMotor2.Set(0.8);
			//		check1=true;
			//		contTimer.Reset();
		//		}

				//if(toteDetector.Get()==false&& contTimer.Get()>1)
				//{
					//intakeMotor.Set(-0.2);
					//intakeMotor2.Set(0.2);
					//check3=true;
				//}

				encoderError= 91-left->GetDistance();
				angleError= angleTarget-heading;
				if(abs(encoderError) < 30)
				{
					throttle = 0.2;
				}
				else
				{
					throttle = 0.6;
				}
				if(abs(encoderError) < 6)
				{
					check3=true;
				}
				//if(abs(encoderError) < 24)
				//{
					//intakeOut.Set(DoubleSolenoid::kForward);
				//}

				lPow = throttle + angleError*0.015;					//getting motor values
				rPow = throttle - angleError*0.015;
				lDrive1.Set(lPow);
				//lDrive2.Set(lPow);
				rDrive1.Set(-rPow);
				//rDrive2.Set(-rPow);


				if(check3==true)
				{
					autoStage=6;
					angleTarget=-140;//-110
					check1=false;
					check2=false;
					check3=false;
					left->Reset();
					intakeOut.Set(DoubleSolenoid::kReverse);
				}
			}

			if(autoStage==6)
			{
				//if(liftDown.Get()==false)
				//{
					//toteLift.Set(DoubleSolenoid::kReverse);
					//intakeOut.Set(DoubleSolenoid::kReverse);
				//}

				angleError= angleTarget-heading;
				lPow = angleError*0.018;					//getting motor values
				rPow = -angleError*0.018;
				if(lPow>0.5)
						{
							lPow=0.5;
							rPow=-0.5;
						}
				if(lPow<-0.5)
				{
					lPow=-0.5;
					rPow=0.5;
				}
				lDrive1.Set(lPow);
				//lDrive2.Set(lPow);
				rDrive1.Set(-rPow);
				//rDrive2.Set(-rPow);
				if(fabs(angleError)<8)
				{
					lDrive1.Set(0);
					//lDrive2.Set(0);
					rDrive1.Set(0);
					//rDrive2.Set(0);
					autoStage=7;
					left->Reset();
					//rEncoder->Reset();
				}
			}

			if(autoStage==7)
			{
				//if(liftDown.Get()==false)
				//{
					//toteLift.Set(DoubleSolenoid::kReverse);
					//intakeOut.Set(DoubleSolenoid::kReverse);
					//intakeMotor.Set(-0.8);
					//intakeMotor2.Set(0.8);
					//check1=true;
			//		contTimer.Reset();
				//}

				if(toteDetector.Get()==false)
					{
						intakeMotor.Set(-0.2);
						intakeMotor2.Set(0.2);
						check2=true;
						//check3=true;
					}

				encoderError= 53-left->GetDistance();
				angleError= angleTarget-heading;

				if(encoderError < 25)
				{
					angleTarget=-100;//-80
				}

				if(abs(encoderError) < 25)
				{
					throttle = 0.2;
				}
				else
				{
					throttle = 0.3;
				}
				if(abs(encoderError) < 13)
				{
					check3=true;
					intakeOut.Set(DoubleSolenoid::kForward);
				}
				//if(abs(encoderError) < 24)
				//{
					//intakeOut.Set(DoubleSolenoid::kForward);
				//}

				lPow = throttle + angleError*0.015;					//getting motor values
				rPow = throttle - angleError*0.015;
				lDrive1.Set(lPow);
				//lDrive2.Set(lPow);
				rDrive1.Set(-rPow);
				//rDrive2.Set(-rPow);


				if(check3==true && check2==true)
				{
					autoStage=8;
					angleTarget=-20;
					check1=false;
					check2=false;
					check3=false;
					left->Reset();
				}
			}
/*
			if(autoStage==8)
			{
				if(toteDetector.Get()==false)
					{
						intakeMotor.Set(-0.2);
						intakeMotor2.Set(0.2);
						check2=true;
						//check3=true;
					}

				//if(liftDown.Get()==false)
				//{
					//toteLift.Set(DoubleSolenoid::kReverse);
					//intakeOut.Set(DoubleSolenoid::kReverse);
				//}

				angleError= angleTarget-heading;
				if(check3==false)
				{
					lPow = -0.35;					//getting motor values
					rPow = 0.35;
					if(lPow>0.5)
							{
								lPow=0.5;
								rPow=-0.5;
							}
					if(lPow<-0.5)
					{
						lPow=-0.5;
						rPow=0.5;
					}
					lDrive1.Set(lPow);
					//lDrive2.Set(lPow);
					rDrive1.Set(-rPow);
				}
				if(fabs(angleError)<5)
				{
					lDrive1.Set(0);
					//lDrive2.Set(0);
					rDrive1.Set(0);
					intakeOut.Set(DoubleSolenoid::kForward);
					//rDrive2.Set(0);
					check3=true;
					left->Reset();
					//rEncoder->Reset();
				}

				if(check2==true && check3==true)
				{
					autoStage=9;
					check1=false;
					check2=false;
					check3=false;
					angleTarget=0;
				}
			}
*/
		//	if(autoStage==8)
			//{
				//angleError= angleTarget-heading;
//				lPow = 0.3;					//getting motor values
	//			rPow = -0.3;
		//		if(lPow>0.45)
			//	{

			//lPow=0.45;
				//	rPow=-0.45;
				//}
				//if(lPow<-0.45)
				//{
				//	lPow=-0.45;
				//	rPow=0.45;
				//}
				//lDrive1.Set(lPow);
				//lDrive2.Set(lPow);
			//	rDrive1.Set(-rPow);
				//rDrive2.Set(-rPow);
				//if(fabs(angleError<8))
				//{
					//lDrive1.Set(0);
					//lDrive2.Set(0);
					//rDrive1.Set(0);
					//rDrive2.Set(0);
					//autoStage=9;
					//angleTarget=-20;
					//left->Reset();
					//rEncoder->Reset();
			//	}
			//}

			if(autoStage==8)
			{
				toteLift.Set(DoubleSolenoid::kForward);
				toteLiftL.Set(DoubleSolenoid::kForward);

				if(check1==false)
				{
					//contTimer.Reset();
					check1=true;
				}

				if(liftDown.Get()==false && encoderError < 50)							//Lift switch
				{
					intakeMotor.Set(1);
					intakeMotor2.Set(-1);
				}

				encoderError= 80-left->GetDistance();
				angleError= angleTarget-heading;
				if(abs(encoderError) < 24)
				{
					throttle = encoderError*0.02;
				}
				else
				{
					throttle = 0.6;
				}

					lPow = throttle + angleError*0.015;					//getting motor values
					rPow = throttle - angleError*0.015;

				lDrive1.Set(lPow);
				//lDrive2.Set(lPow);
				rDrive1.Set(-rPow);
				//rDrive2.Set(-rPow);
			}


			SmartDashboard::PutNumber("IMU Angle",imu->GetYaw());
			SmartDashboard::PutNumber("IMU Pitch",imu->GetPitch());
			SmartDashboard::PutNumber("IMU Roll",imu->GetRoll());
			SmartDashboard::PutNumber("heading",heading);
			SmartDashboard::PutNumber("lPow",lPow);
			SmartDashboard::PutNumber("rPow",rPow);
			SmartDashboard::PutNumber("AutoStage",autoStage);
			SmartDashboard::PutNumber("angleTarget",angleTarget);
			SmartDashboard::PutNumber("angleError",angleError);
			SmartDashboard::PutNumber("encoderTarget",encoderTarget);
			SmartDashboard::PutNumber("encoderError",encoderError);
			SmartDashboard::PutNumber("Left Distance", left-> GetDistance());
			SmartDashboard::PutBoolean("Tote Detector",toteDetector.Get());
			SmartDashboard::PutBoolean("LiftDown",liftDown.Get());
			SmartDashboard::PutBoolean("Check1",check1);
			SmartDashboard::PutBoolean("Check2",check2);
			SmartDashboard::PutBoolean("Check3",check3);

		  	  }
			  else if(autoToDo == 2){
				  if(autoStage == 0)
				  			{
				  			autoStage=1;
				  			//Start contLift
				  			contGripper.Set(DoubleSolenoid::kReverse);                 //walawala
				  			//Start Intake
				  			intakeMotor.Set(-0.8);            						//Motor Direction
				  			intakeMotor2.Set(0.8);
				  			//Start Intake
				  			whatever.Start();
				  			whatever.Reset();
				  			intakeOut.Set(DoubleSolenoid::kForward);
				  			}

				  			if(autoStage==1)
				  			{
				  				if(whatever.Get()>0.3 && whatever.Get()<autoHeightTime1)
				  				{
				  					contLift.Set(-0.8);										   //Speed
				  					liftLock.Set(DoubleSolenoid::kReverse);
				  				}
				  				if(whatever.Get()>autoHeightTime1)
				  				{
				  					contLift.Set(0);
				  					liftLock.Set(DoubleSolenoid::kForward);
				  					check4=true;
				  					//check1=true;
				  				}

				  				if(toteDetector.Get()==false)						//Tote detect
				  				{
				  					toteLift.Set(DoubleSolenoid::kForward);
				  					toteLiftL.Set(DoubleSolenoid::kForward);
				  					intakeMotor.Set(-0.2);
				  					intakeMotor2.Set(0.2);


				  				}

				  				if(liftDown.Get()==false)							//Can be improved
				  				{
				  					check3=true;
				  				}
				  				if(check3==true && check4 ==true)
				  				{
				  					toteLift.Set(DoubleSolenoid::kReverse);
				  					intakeOut.Set(DoubleSolenoid::kReverse);
				  					toteLiftL.Set(DoubleSolenoid::kReverse);
				  					autoStage=2;
				  					check1=false;
				  					check2=false;
				  					check3=false;
				  					check4=false;
				  				}
				  			}

				  			if(autoStage == 2){
				  				if(check1 == false){
				  					heading = imu->GetYaw()+90;
				  					check1 = true;
				  				}

				  				lPow = (heading - imu->GetYaw())*.005 + .2;
				  				rPow = (heading - imu->GetYaw())*.005 + .2;
				  				lDrive1.Set(lPow);
				  				rDrive1.Set(rPow);

				  				if(abs(imu->GetYaw() - 90) < 3){
				  					left->Reset();
				  					autoStage = 3;
				  				}
				  			}

				  			if(autoStage == 3){
				  				while(left->GetDistance() < 110){
				  					lDrive1.Set(.5);
				  					rDrive1.Set(-.5);
				  					SmartDashboard::PutNumber("Distance", left->GetDistance());
				  				}
				  				if(left->GetDistance() >= 110){
				  					lDrive1.Set(0);
				  					rDrive1.Set(0);
				  					autoStage = 4;
				  				}
				  			}
				  			if(autoStage == 4){
				  				lDrive1.Set(.4);
				  				rDrive1.Set(.4);
				  				Wait(1);
				  				lDrive1.Set(0);
				  				rDrive1.Set(0);
				  				autoStage++;
				  			}

				  			SmartDashboard::PutNumber("IMU Angle",imu->GetYaw());


			  	}

			  else if(autoToDo == 3){
				  if(autoTimer.Get()<1)
				  {
					  contHook.Set(DoubleSolenoid::kForward);
				  }

				 // if(autoTimer.Get() < .5){ lDrive1.Set(.33); rDrive1.Set(-.33); }
				  if(autoTimer.Get() > 1 && left->GetDistance() > -50)
				  {
					  contHook.Set(DoubleSolenoid::kReverse);
					  lDrive1.Set(-.4);
					  rDrive1.Set( .4);
				  }
				  else{
					  lDrive1.Set(0);
					  rDrive1.Set(0);
				  }
			  }
		  	  else{
		  			 lDrive1.Set(0);
		  			 rDrive1.Set(0);
	  }
	  SmartDashboard::PutNumber("autostage", autoStage);
	  SmartDashboard::PutNumber("Left distance", left->GetDistance());
	}
	}
	void OperatorControl() {

		while (IsOperatorControl() && IsEnabled()) {

			SmartDashboard::PutNumber("Yaw", imu->GetYaw());
			liftPow = -adjust(Mikey.GetRawAxis(1));	//function(see adjustValues.cpp)
			contLift.Set(liftPow);		//checks where it is supposed to be goign and sets it
			if(liftPow == 0){ liftLock.Set(DoubleSolenoid::kForward); }		//prevent backdrive
			else { liftLock.Set(DoubleSolenoid::kReverse);}		//when it isn't moving lock

			if(Mikey.GetRawButton(6)){
				contGripper.Set(DoubleSolenoid::kForward);
			}
			else if(Mikey.GetRawButton(5)){
				contGripper.Set(DoubleSolenoid::kReverse);
			}

			//The following section is for the drive train
			throttle = -adjust(James.GetRawAxis(1));		//placing deadbands on throttle
			turn = .75*adjust(James.GetRawAxis(4));				//and turn
						//getting motor values


		//	if(fabs(James.GetRawAxis(4))>.2){		//you need this so it only triggers the Nav X when it isn't
				//loopCoJamest = 0;						//resets when it resets yaw
				lPow = leftPower(throttle, turn);					//getting motor values
				rPow = rightPower(throttle, turn);
		/*	}
			else if(fabs(James.GetRawAxis(4))<.2){	//again so it only triggers when
				float past = current;
				float current = imu->GetYaw();
				loopCount++;	//so it only resets it 15 times
				if(loopCount < 15){	startAngle = imu->GetYaw(); }//for resetting start angle
				iE = integrate(past, current, iE, startAngle, iC, wait, true);
				tE = PIDify(past, current, startAngle, pC, iE, dC, wait, true);
				SmartDashboard::PutNumber("Desired heading", startAngle);
				lPow = NavXL(throttle, tE);		//applies the error
				rPow = NavXR(throttle, tE);
			}*/

			if(James.GetRawButton(1)){
				modifier = .6;
				lastModifier = .6;
			}
			else if(James.GetRawButton(2)){
				modifier = .4;
				lastModifier = .4;
			}
			else if(James.GetRawButton(3)){
				modifier = 1;
				lastModifier = 1;
			}

			if((toteTime2.Get() < .5 || !toteDetector.Get()) && !toteManual){ modifier = .3; }	//new
			else { modifier = lastModifier; }	//new

			lDrive1.Set(lPow * modifier);
			rDrive1.Set(rPow*modifier);
			//This is for totes
			if(Mikey.GetRawButton(3)){	//toggles between whether or not you have manual control
				toteManual = true;		//toggles it
				toteTime.Reset();
				toteLoopOne = true;
			}
			if(Mikey.GetRawButton(2)){
			 	toteManual = false;
			 	toteLoopOne = false;
			 	triggered = false;
			 	toteTime.Reset();
			}
			if(toteManual){		//if you have control
				if(Mikey.GetPOV() == 180){						//simple hit it to go up or down
					toteLift.Set(DoubleSolenoid::kForward);
					toteLiftL.Set(DoubleSolenoid::kForward);
				}
				if(Mikey.GetPOV() == 0){
					toteLift.Set(DoubleSolenoid::kReverse);
					toteLiftL.Set(DoubleSolenoid::kReverse);
				}
			}
			if(!toteManual){	//if auto
				if(intakeVal == 0 && toteDetector.Get()){
					intakeMotor.Set(-1);
					intakeMotor2.Set(1);
				}
			//	else if(!toteDetector.Get()){
			//			intakeMotor.Set(0);
			//			intakeMotor2.Set(0);
			//	}
				if(!toteDetector.Get() && !toteLoopOne){
					toteTime.Reset();
					toteTime2.Reset();
					//toteLift.Set(DoubleSolenoid::kForward);
					//toteLiftL.Set(DoubleSolenoid::kForward);
					toteLoopOne = true;
					triggered = false;
				}
				if(toteTime.Get() > .3 && !triggered && !toteDetector.Get()){
					toteLift.Set(DoubleSolenoid::kForward);
					toteLiftL.Set(DoubleSolenoid::kForward);
					triggered= true;
				}
				if(!liftDown.Get()){
					toteLift.Set(DoubleSolenoid::kReverse);
					toteLiftL.Set(DoubleSolenoid::kReverse);
					toteTime2.Reset();	//new
				}

				if(toteDetector.Get() && toteTime.Get() > 1.5){
					toteLoopOne = false;
				}
			}

			if(Mikey.GetRawButton(7)){
				intakeOut.Set(DoubleSolenoid::kForward);
			}
			else if(Mikey.GetRawButton(8)){
				intakeOut.Set(DoubleSolenoid::kReverse);
			}

			intakeVal = adjust(Mikey.GetRawAxis(3));
			//if(intakeVal > 0){ intakeVal *= .7; }
			if (intakeVal < 0){ intakeVal *= .4; }
			if(intakeVal != 0 || toteManual == true){
				intakeMotor.Set(-intakeVal);
				intakeMotor2.Set(intakeVal);
			}

			if(Mikey.GetRawButton(4)){
				coOp.Set(DoubleSolenoid::kForward);
			}
			if(Mikey.GetRawButton(1)){
				coOp.Set(DoubleSolenoid::kReverse);
				toteManual = false;
				toteLift.Set(DoubleSolenoid::kReverse);
				toteLiftL.Set(DoubleSolenoid::kReverse);
			}

			if(James.GetRawButton(5)){
				contHook.Set(DoubleSolenoid::kForward);
			}
			else if(James.GetRawButton(6)){
				contHook.Set(DoubleSolenoid::kReverse);
			}

			SmartDashboard::PutBoolean("Tote loop one", toteLoopOne);
			SmartDashboard::PutBoolean("Tote Detector", toteDetector.Get());
			SmartDashboard::PutBoolean("Lift Down", liftDown.Get());
			SmartDashboard::PutBoolean("Tote Manual", toteManual);
			SmartDashboard::PutBoolean("Left Step Grabber", leftHook.Get());
			SmartDashboard::PutBoolean("Right Step Grabber", rightHook.Get());
			SmartDashboard::PutNumber("Drive modifier", modifier);

		}
	}


};

START_ROBOT_CLASS(Robot);
