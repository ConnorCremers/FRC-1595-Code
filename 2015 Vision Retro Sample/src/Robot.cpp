#include "WPILib.h"
#include <vector>
#include <cmath>

/**
 * Example of finding yellow totes based on retroreflective target.
 * This example utilizes an image file, which you need to copy to the roboRIO
 * To use a camera you will have to integrate the appropriate camera details with this example.
 * To use a USB camera instead, see the IntermediateVision example for details
 * on using the USB camera. To use an Axis Camera, see the AxisCamera example for details on
 * using an Axis Camera.
 *
 * Sample images can be found here: http://wp.wpi.edu/wpilib/2015/01/16/sample-images-for-vision-projects/ 
 */
class VisionRetro2015Sample : public SampleRobot
{
	//A structure to hold measurements of a particle
	struct ParticleReport {
		double PercentAreaToImageArea;
		double Area;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;
	};

	//Structure to represent the scores for the various tests used for target identification
	struct Scores {
		double Area;
		double Aspect;
	};

	//Images
	Image *frame;
	Image *binaryFrame, *oppBinaryFrame;
	int imaqError;
	Point point;
	PixelValue value;

	  float grayscale = 0;  // A grayscale pixel value.

	  int xCoor;
	  int yCoor;


	IMAQdxSession session;
	IMAQdxError imaqError2;
	//Constants
	Range RING_HUE_RANGE = {0, 255};	//Default hue range for yellow tote
	Range RING_SAT_RANGE = {0, 255};	//Default saturation range for yellow tote
	Range RING_VAL_RANGE = {215, 255};	//Default value range for yellow tote
	Range oppValRange = {0, 215};
	int rangeCutOff;
	double AREA_MINIMUM = 0.5; //Default Area minimum for particle as a percentage of total image area
	double LONG_RATIO = 2.22; //Tote long side = 26.9 / Tote height = 12.1 = 2.22
	double SHORT_RATIO = 1.4; //Tote short side = 16.9 / Tote height = 12.1 = 1.4
	double SCORE_MIN = 75.0;  //Minimum score to be considered a tote
	double VIEW_ANGLE = 49.4; //View angle fo camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480
	ParticleFilterCriteria2 criteria[1];
	ParticleFilterOptions2 filterOptions = {0,1,1,1};
	BorderMethod method = IMAQ_BORDER_CLEAR;
	Scores scores;
	ScalingMode scaling = IMAQ_SCALE_SMALLER;
	Rect scalingTest = {1, 1, 400, 600};
	int triHeight;
	int triLength;
	float angle;


public:
	void RobotInit() override {
	    // create images
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		binaryFrame = imaqCreateImage(IMAQ_IMAGE_U8, 0);
		oppBinaryFrame = imaqCreateImage(IMAQ_IMAGE_U8, 0);

		imaqError2 = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &session);
		if(imaqError2 != IMAQdxErrorSuccess) {
		DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long)imaqError2) + "\n");
		}
		imaqError2 = IMAQdxConfigureGrab(session);
		if(imaqError2 != IMAQdxErrorSuccess) {
		DriverStation::ReportError("IMAQdxConfigureGrab error: " + std::to_string((long)imaqError2) + "\n");
		}

	}

	void Autonomous() override {
		IMAQdxStartAcquisition(session);
		value = {grayscale};
		while (IsAutonomous() && IsEnabled())
		{
			IMAQdxGrab(session, frame, true, NULL);
			if(imaqError2 != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long)imaqError2) + "\n");
			} else {
			//CameraServer::GetInstance()->SetImage(frame);
			}
			//Threshold the image looking for ring light color
			SmartDashboard::GetNumber("Value? ", rangeCutOff);
			oppValRange.maxValue = rangeCutOff;
			RING_VAL_RANGE.minValue = rangeCutOff;
			imaqError = imaqColorThreshold(binaryFrame, frame, 255, IMAQ_HSV, &RING_HUE_RANGE, &RING_SAT_RANGE, &RING_VAL_RANGE);
			imaqError = imaqColorThreshold(oppBinaryFrame, frame, 255, IMAQ_HSV, &RING_HUE_RANGE, &RING_SAT_RANGE, &oppValRange);
			//CameraServer::GetInstance()->SetImage(binaryFrame);
			//Sets the values of outermost pixels
			for(yCoor = 1; yCoor < 481; yCoor++){
				point = {1,yCoor};
				imaqSetPixel(binaryFrame, point, value);
				imaqSetPixel(oppBinaryFrame, point, value);
				point = {639,yCoor};
				imaqSetPixel(binaryFrame, point, value);
				imaqSetPixel(oppBinaryFrame, point, value);
			}

			for(xCoor = 1; xCoor < 6410; xCoor++){
				point = {xCoor,1};
				imaqSetPixel(binaryFrame, point, value);
				imaqSetPixel(oppBinaryFrame, point, value);
				point = {xCoor,479};
				imaqSetPixel(binaryFrame, point, value);
				imaqSetPixel(oppBinaryFrame, point, value);
			}
			CameraServer::GetInstance()->SetImage(binaryFrame);
			//Send particle count to dashboard
			int numParticles = 0;
			int numParticles2 = 0;
			imaqError = imaqCountParticles(binaryFrame, 1, &numParticles);
			SmartDashboard::PutNumber("Masked particles", numParticles);

			//filter out small particles
			float areaMin = SmartDashboard::GetNumber("Area min %", AREA_MINIMUM);
			criteria[0] = {IMAQ_MT_AREA_BY_IMAGE_AREA, areaMin, 100, false, false};
			imaqError = imaqParticleFilter4(oppBinaryFrame, oppBinaryFrame, criteria, 1, &filterOptions, NULL, NULL);
			imaqError = imaqParticleFilter4(binaryFrame, binaryFrame, criteria, 1, &filterOptions, NULL, NULL);
			//SendToDashboard(binaryFrame, imaqError);
			//Send particle count after filtering to dashboard
			imaqError = imaqCountParticles(binaryFrame, 1, &numParticles);
			SmartDashboard::PutNumber("Filtered particles", numParticles);
			imaqError = imaqCountParticles(oppBinaryFrame, 1, &numParticles2);

			if(numParticles > 0 && numParticles2 > 0) {
				//Measure particles and sort by particle size
				std::vector<ParticleReport> particles;
				for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
				{
					ParticleReport par;
					imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_LEFT, &(par.BoundingRectLeft));
					imaqMeasureParticle(binaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_BOTTOM, &(par.BoundingRectBottom));
					imaqMeasureParticle(binaryFrame, particleIndex,0, IMAQ_MT_BOUNDING_RECT_RIGHT, &(par.BoundingRectRight));
					particles.push_back(par);
				}
				sort(particles.begin(), particles.end(), CompareParticleSizes);

				std::vector<ParticleReport> particles2;
				for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
				{
					ParticleReport par2;
					imaqMeasureParticle(oppBinaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_TOP, &(par2.BoundingRectTop));
					particles2.push_back(par2);
				}
				sort(particles2.begin(), particles2.end(), CompareParticleSizes);

				triHeight = GetBot(particles.at(0)) - GetTop(particles2.at(0));
				triLength = GetRight(particles.at(0)) - GetLeft(particles.at(0));
				angle = atan2(triHeight, triLength) *180/3.1415;

				SmartDashboard::PutNumber("Triangle Height: ", triHeight);
				SmartDashboard::PutNumber("Triangle Length: ", triLength);
				SmartDashboard::PutNumber("Angle: ", angle);
			/*	SmartDashboard::PutNumber("Left boundary", GetLeft(particles.at(0)));
				SmartDashboard::PutNumber("Right boundary", GetRight(particles.at(0)));
				SmartDashboard::PutNumber("Bot boundary", GetBot(particles.at(0)));
				SmartDashboard::PutNumber("Top boundary", GetTop(particles.at(0)));*/
			} else {
		//		SmartDashboard::PutBoolean("IsTarget", false);
			}

			Wait(0.005);				// wait for a motor update time
		}
		IMAQdxStopAcquisition(session);
	}

	void OperatorControl() override {
		while(IsOperatorControl() && IsEnabled()) {
			Wait(0.005);				// wait for a motor update time
		}
	}


	//Send image to dashboard if IMAQ has not thrown an error
	void SendToDashboard(Image *image, int error)
	{
		if(error < ERR_SUCCESS) {
			DriverStation::ReportError("Send To Dashboard error: " + std::to_string((long)imaqError) + "\n");
		} else {
			CameraServer::GetInstance()->SetImage(binaryFrame);
		}
	}

	//Comparator function for sorting particles. Returns true if particle 1 is larger
	static bool CompareParticleSizes(ParticleReport particle1, ParticleReport particle2)
	{
		//we want descending sort order
		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
	}

	/**
	 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
	 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
	 */
	double ratioToScore(double ratio)
	{
		return (fmax(0, fmin(100*(1-fabs(1-ratio)), 100)));
	}


	double AreaScore(ParticleReport report)
	{
		double boundingArea = (report.BoundingRectBottom - report.BoundingRectTop) * (report.BoundingRectRight - report.BoundingRectLeft);
		//Tape is 7" edge so 49" bounding rect. With 2" wide tape it covers 24" of the rect.
		return ratioToScore((49/24)*report.Area/boundingArea);
	}

	int GetRight(ParticleReport report){
		return(report.BoundingRectRight);
	}
	int GetLeft(ParticleReport report){
			return(report.BoundingRectLeft);
	}
	int GetTop(ParticleReport report){
			return(report.BoundingRectTop);
	}
	int GetBot(ParticleReport report){
		return(report.BoundingRectBottom);
	}
	/**
	 * Method to score if the aspect ratio of the particle appears to match the retro-reflective target. Target is 7"x7" so aspect should be 1
	 */
	double AspectScore(ParticleReport report)
	{
		return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop)));
	}


	/**
	 * Computes the estimated distance to a target using the width of the particle in the image. For more information and graphics
	 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
	 *
	 * @param image The image to use for measuring the particle estimated rectangle
	 * @param report The Particle Analysis Report for the particle
	 * @return The estimated distance to the target in feet.
	 */
	double computeDistance (Image *image, ParticleReport report) {
		double normalizedWidth, targetWidth;
		int xRes, yRes;

		imaqGetImageSize(image, &xRes, &yRes);
		normalizedWidth = 2*(report.BoundingRectRight - report.BoundingRectLeft)/xRes;
		SmartDashboard::PutNumber("Width", normalizedWidth);
		targetWidth = 7;

		return  targetWidth/(normalizedWidth*12*tan(VIEW_ANGLE*M_PI/(180*2)));
	}
};

START_ROBOT_CLASS(VisionRetro2015Sample)
