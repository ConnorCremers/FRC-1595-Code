#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <math.h>
#include <iostream>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
using namespace cv;
using namespace std;


const int w = 500;
int levels = 3;

vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

void on_low_r_thresh_trackbar(int, void *);
void on_high_r_thresh_trackbar(int, void *);
void on_low_g_thresh_trackbar(int, void *);
void on_high_g_thresh_trackbar(int, void *);
void on_low_b_thresh_trackbar(int, void *);
void on_high_b_thresh_trackbar(int, void *);

int low_r = 100, low_g = 200, low_b = 200;
int high_r = 255, high_g = 255, high_b = 255;
int outputs [ 4 ];
int targetPixel;
RNG rng(12345);
VideoCapture cap, cap2;

bool setup = false;

int whichProg[4];

int main()
{
	int coms;
	if (!setup) {
		wiringPiSetup ();
		cout << "Wiring pi setup complete\n";		
		if((coms = serialOpen("/dev/serial0", 115200)) < 0){
		fprintf(stderr,"unable to open serial device: %s\n",strerror(errno));}
		setup = true;
		cout << coms << "     setup complete" << endl;
	}
	//cap.release();
	cap.open(0);
	
	cap2.open("/home/pi/Desktop/peg.mp4");
	whichProg[0] = 1;
cap.set(CV_CAP_PROP_FPS,10);
	//	string imageName("../Boiler.jpg"); // by default

	Mat image;
	fflush(stdout);
	while (true) {
		if(serialDataAvail(coms)){
			whichProg[0] = serialGetchar(coms);
		}
		//image = imread(imageName.c_str(), IMREAD_COLOR); // Read the file
		if(whichProg[0] == 1){
			cap.read(image);
		}
		else if(whichProg[0] == 2){
			cap2.read(image);
		}

		if (image.empty())                      // Check for invalid input
		{
			cout << "Could not open or find the image" << std::endl;
			return -1;
		}
		resize(image, image, Size(640, 480), 0, 0, INTER_CUBIC);

		namedWindow("Object Detection");


		createTrackbar("Low R", "Object Detection", &low_r, 255, on_low_r_thresh_trackbar);
		createTrackbar("High R", "Object Detection", &high_r, 255, on_high_r_thresh_trackbar);
		createTrackbar("Low G", "Object Detection", &low_g, 255, on_low_g_thresh_trackbar);
		createTrackbar("High G", "Object Detection", &high_g, 255, on_high_g_thresh_trackbar);
		createTrackbar("Low B", "Object Detection", &low_b, 255, on_low_b_thresh_trackbar);
		createTrackbar("High B", "Object Detection", &high_b, 255, on_high_b_thresh_trackbar);

		Mat thresholded;
		inRange(image, Scalar(low_b, low_g, low_r), Scalar(high_b, high_g, high_r), thresholded);
		imshow("thresholded", thresholded);
		//	waitKey(0);
		findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

		vector<vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect(contours.size());

		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
			boundRect[i] = boundingRect(Mat(contours_poly[i]));
		}
		Size imagesize = thresholded.size();
		Mat drawing = Mat::zeros(thresholded.size(), CV_8UC3);

		vector<Rect> possibles(contours.size());
		int possibleCounter = 0;
		for (int i = 0; i < contours.size(); i++) {
			int area = boundRect[i].width*boundRect[i].height;
			if (area > 500) {
				possibles[possibleCounter] = boundRect[i];
				possibleCounter++;
			}
		}

		int j = 0;
		bool target = false;

		while (j < possibleCounter && !target) {
			for (int k = 0; k < possibleCounter; k++) {

				if (fabs(possibles[j].y - possibles[k].y) / possibles[j].height < .5 && abs(possibles[j].y - possibles[k].y) < 80 && k != j) {
					if (abs(possibles[j].width - possibles[k].width) < 50 && abs(possibles[j].height - possibles[k].height) < 40) {
						if (fabs(fabs(possibles[j].x - possibles[k].x) / (possibles[j].width + possibles[k].width) - 1.8) < 1.25) {
							if (fabs(possibles[j].height / possibles[j].width - 3) < 1.75) {
								if (((abs(possibles[j].x - possibles[k].x) + possibles[j].width) / possibles[j].height - 1.5) < .75) {
									if (fabs(possibles[j].width / (fabs(possibles[j].x - possibles[k].x) + possibles[j].width) - .25) < .2) {
										float temp = possibles[j].width / (fabs(possibles[j].x - possibles[k].x) + possibles[j].width) - .25;
										
										Scalar color = Scalar(255, 0, 0);
										rectangle(drawing, possibles[k].tl(), possibles[k].br(), color, 2, 8, 0);

										rectangle(drawing, possibles[j].tl(), possibles[j].br(), color, 2, 8, 0);
										target = true;

										if (possibles[j].x > possibles[k].x) {	targetPixel = (possibles[k].x + possibles[k].width * 2.5 + possibles[j].x - possibles[j].width * 1.5)/2; }
										else { targetPixel = (possibles[j].x + possibles[j].width * 2.5 + possibles[k].x - possibles[k].width * 1.5)/2; }
										
										Point top = Point(targetPixel, 5);
										Point bottom = Point(targetPixel, 400);
										line(image, top, bottom, Scalar(0, 0, 255));
											
											if(targetPixel >= 765){
											serialPutchar(coms, 255); serialPutchar(coms, 255); serialPutchar(coms, 255);	
												//cout << "765" << endl;											
											}

											else if(targetPixel >= 510){
												serialPutchar(coms, 255); serialPutchar(coms, 255); serialPutchar(coms,targetPixel-255);
												cout << targetPixel-510 << endl;
											}
											else if(targetPixel >= 255){
												serialPutchar(coms, 255); serialPutchar(coms, targetPixel-255); serialPutchar(coms, 0);
												cout << targetPixel-255 << endl;
											}
											else{
												cout << targetPixel << endl;
												serialPutchar(coms, targetPixel); serialPutchar(coms, 0); serialPutchar(coms, 0);
											}
										
									}
								}
							}
						}
					}
				}
			}
			j++;
		}
		namedWindow("image");
		imshow("image", image);

		namedWindow("Contours", WINDOW_AUTOSIZE);
		imshow("Contours", drawing);

		waitKey(1);
	}
	/*	createTrackbar("levels+3", "contours", &levels, 7, on_trackbar);

	on_trackbar(0, 0);*/
	waitKey();

	return 0;
}

void on_low_r_thresh_trackbar(int, void *)
{
	low_r = min(high_r - 1, low_r);
	setTrackbarPos("Low R", "Object Detection", low_r);
}
void on_high_r_thresh_trackbar(int, void *)
{
	high_r = max(high_r, low_r + 1);
	setTrackbarPos("High R", "Object Detection", high_r);
}
void on_low_g_thresh_trackbar(int, void *)
{
	low_g = min(high_g - 1, low_g);
	setTrackbarPos("Low G", "Object Detection", low_g);
}
void on_high_g_thresh_trackbar(int, void *)
{
	high_g = max(high_g, low_g + 1);
	setTrackbarPos("High G", "Object Detection", high_g);
}
void on_low_b_thresh_trackbar(int, void *)
{
	low_b = min(high_b - 1, low_b);
	setTrackbarPos("Low B", "Object Detection", low_b);
}
void on_high_b_thresh_trackbar(int, void *)
{
	high_b = max(high_b, low_b + 1);
	setTrackbarPos("High B", "Object Detection", high_b);
}