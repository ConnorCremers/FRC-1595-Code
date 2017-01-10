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




	IMAQdxSession session;
	IMAQdxError imaqError2;
	//Constants
	//Images
	Image *frame;
	Image *binaryFrame, *oppBinaryFrame;
	int imaqError;
	Point point;
	PixelValue value;
	Range RING_HUE_RANGE = {0, 255};	//Default hue range for yellow tote
	Range RING_SAT_RANGE = {0, 255};	//Default saturation range for yellow tote
	Range RING_VAL_RANGE = {215, 255};	//Default value range for yellow tote
	Range oppValRange = {0, 215};
	int rangeCutOff = 215;
	double AREA_MINIMUM = 0.5; //Default Area minimum for particle as a percentage of total image area
	ParticleFilterCriteria2 criteria[1];
	ParticleFilterOptions2 filterOptions = {0,1,1,1};
	float grayscale = 0;  // A grayscale pixel value.
	int xCoor;
	int yCoor;
	int triHeight;
	int triLength;
	float angle;


public:
	void RobotInit() override {
	    // create images
		SmartDashboard::PutNumber("Value? ", rangeCutOff);
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
			for(yCoor = 0; yCoor < 481; yCoor++){
				point = {0,yCoor};
				imaqSetPixel(binaryFrame, point, value);
				imaqSetPixel(oppBinaryFrame, point, value);
				point = {639,yCoor};
				imaqSetPixel(binaryFrame, point, value);
				imaqSetPixel(oppBinaryFrame, point, value);
			}

			for(xCoor = 0; xCoor < 641; xCoor++){
				point = {xCoor,0};
				imaqSetPixel(binaryFrame, point, value);
				imaqSetPixel(oppBinaryFrame, point, value);
				point = {xCoor,479};
				imaqSetPixel(binaryFrame, point, value);
				imaqSetPixel(oppBinaryFrame, point, value);
			}
			//Send particle count to dashboard
			int numParticles = 0;
			int numParticles2 = 0;
			imaqError = imaqCountParticles(binaryFrame, 1, &numParticles);
			CameraServer::GetInstance()->SetImage(binaryFrame);
			//filter out small particles
			float areaMin = SmartDashboard::GetNumber("Area min %", AREA_MINIMUM);
			criteria[0] = {IMAQ_MT_AREA_BY_IMAGE_AREA, 1, 100, false, false};
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
			} else {
				triHeight = 0;
				triLength = 0;
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


};

START_ROBOT_CLASS(VisionRetro2015Sample)
