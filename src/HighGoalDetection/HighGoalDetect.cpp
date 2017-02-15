#include "zhelpers.hpp"
#include <zmq.hpp>
#include <string>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;
using namespace zmq;
using namespace std;

/* IMPORTANT IMPORTANT
   RUN v4l2-ctl -d /dev/video1 -c exposure_auto=1 -c exposure_absolute=10
   maybe not exposure_auto_priority=0 -c 
   RUN v4l2-ctl -d /dev/video1 -c white_balance_temperature_auto=0
*/
#define PI 3.14159265

//FIELD SPECS
const int DIST_BETWEEN_GOAL_TARGETS = 7; //inches
const int TOP_TARGET_HEIGHT = 88; //inches, top of tape
const int BOTTOM_TARGET_HEIGHT = 78; //inches, bottom of tape

//CAMERA SPECS
const double DIAGONAL_FOV = 68.5; //degrees
const double HORIZONTAL_FOV = 45.0; //degrees was 61.39
const double VERTICAL_FOV = 36.0; //degrees
const double FOCAL_LENGTH = 539.0485745586101; //pixelzzz
const int PIXEL_WIDTH = 640; //pixelzzz
const int PIXEL_HEIGHT = 480; //pixelzzz
const double CENTER_LINE = 319.5; //pixelzzz
//CHECK BELOW
const double CAMERA_ANGLE = 0.0; //degrees
const double CAMERA_HEIGHT = 6.0; //inches
const double OFFSET_TO_FRONT = 0.0; //inches

string WINDOW_NAME = "High Goal Window";

const int LOWER_GREEN_HUE = 30;
const int LOWER_GREEN_SAT = 60; 
const int LOWER_GREEN_VAL = 60;

const int UPPER_GREEN_HUE = 130;
const int UPPER_GREEN_SAT = 255;
const int UPPER_GREEN_VAL = 255;

const double CAP_SAT = 0.5;

const int MIN_AREA = 3000; //was 2000
RNG rng(12345);

int main() {
	cout << "Built with OpenCV B-) " << CV_VERSION << endl;
	Mat image;
	VideoCapture capture(1);
//	capture.set(CV_CAP_PROP_EXPOSURE, 0.0);
	capture.set(CV_CAP_PROP_SATURATION, CAP_SAT);

 	if (!capture.isOpened()) {
	    	cout << "!!! Failed to open camera darn :((((( " << endl;
    		return -1;
  	}

	Mat frame;

	for(;;) {
//READING
		if (!capture.read(frame)) {
			break;
		}

		Mat cloneImage = frame.clone();
		imshow(WINDOW_NAME, cloneImage);
//STARTING PROCESSING
		medianBlur(frame,frame, 5);

		Mat hsvImage;
		cvtColor(frame, hsvImage, COLOR_BGR2HSV);

		Mat greenRange;
		inRange(hsvImage, Scalar(LOWER_GREEN_HUE, LOWER_GREEN_SAT, LOWER_GREEN_VAL), Scalar(UPPER_GREEN_HUE, UPPER_GREEN_SAT, UPPER_GREEN_VAL), greenRange);

		GaussianBlur(greenRange, greenRange, Size(9,9), 2,2);

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours (greenRange, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
		vector<vector<Point> > contours_poly(contours.size() );
		vector<Rect> boundRect(contours.size() );

		int upRect;
		int downRect;

		for(int c = 0; c < contours.size(); c++) {
			Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) );
			if (contourArea(contours[c]) < MIN_AREA) {
				continue;
			}
			drawContours(cloneImage, contours, c, color, 2, 8, hierarchy, 0, Point() );
			approxPolyDP( Mat(contours[c]), contours_poly[c], 3, true);
			boundRect[c] = boundingRect( Mat(contours_poly[c]) );
			//double aspectRatio = (boundRect[c].br().x - boundRect[c].tl().x)/(boundRect[c].br().y - boundRect[c].tl().y);
//Be careful for aspectRatio
			rectangle(cloneImage, boundRect[c].tl(), boundRect[c].br(), color, 2, 8, 0);

		}
		
		for(int c = 0; c < boundRect.size();) {
			cout << "bound rect area: " << boundRect[c].area() << endl;
			if (boundRect[c].area() < MIN_AREA) {
				boundRect.erase(boundRect.begin() + c);
				continue;
			}
			c++;
		}
		
		if (boundRect.size() < 2) {
			cout << "no rectangles :(" << endl;
			continue;
		}

		if (boundRect[0].br().y < boundRect[1].br().y) {
			upRect = 0;
			downRect = 1;
		} else {
			upRect = 1;
			downRect = 0;
		}
		double centerOfTargets = (boundRect[upRect].tl().x + boundRect[downRect].br().x) / 2.0;
//FINDING ANGLE
		double angleToMoveApprox;
		angleToMoveApprox = (centerOfTargets - CENTER_LINE) * HORIZONTAL_FOV / PIXEL_WIDTH;
		cout << "approx angle: " << angleToMoveApprox << endl;
		cout << "pixel distance for centers: " << centerOfTargets - CENTER_LINE << endl;

		double angleToMoveAgain;
		angleToMoveAgain = atan((centerOfTargets - CENTER_LINE) / FOCAL_LENGTH);
		angleToMoveAgain = (angleToMoveAgain *180/PI); //converts radients to degrees
		cout << "angleToMoveAgain: " << angleToMoveAgain << endl;

		double distanceToTopOfGoal;
		double y = boundRect[downRect].br().y + (boundRect[upRect].tl().y - boundRect[downRect].br().y) / 2.0;
		y = -(2 *y / PIXEL_HEIGHT -1);
		distanceToTopOfGoal = (TOP_TARGET_HEIGHT - CAMERA_HEIGHT) / tan((y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE) * PI / 180);
		cout << "distance to top goal: " << distanceToTopOfGoal << endl;

		imshow(WINDOW_NAME, cloneImage);

		char key = cvWaitKey (10); 
		if (key == 27) { //Esc
			break;
		}
	}
	return 0;
}
