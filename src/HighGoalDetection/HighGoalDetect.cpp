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
 RUN v4l2-ctl -d /dev/video2 -c exposure_auto=1 -c exposure_absolute=5 -c brightness=0 -c white_balance_temperature_auto=0

   hello lili is here
 */
#define PI 3.14159265

//FIELD SPECS
const double DIST_BETWEEN_GOAL_TARGETS = 7.0; //inches
const double TOP_TARGET_HEIGHT = 88.0; //inches, top of tape
const double BOTTOM_TARGET_HEIGHT = 78.0; //inches, bottom of tape
const double WIDTH_OF_GOAL = 15.0; //inches
const double HEIGHT_DIFF_OF_TOP_OF_TARGETS = 8.0; // TO SUBTRACT LATER
const double HEIGHT_OF_TOP_OF_TOP_TARGET = 88.0;
const double HEIGHT_OF_TOP_OF_BOTTOM_TARGET = 80.0;

//CAMERA SPECS
const double DIAGONAL_FOV = 68.5; //degrees
const double HORIZONTAL_FOV = 59.7; //60.0; //45.0; //degrees was 61.39
const double VERTICAL_FOV = 36.5; //33.58; //36.0; //degrees
const double FOCAL_LENGTH = 340; //500.0; //1170.0; // 239.369; //pixelzzz was 539.0485745586101
const double PIXEL_WIDTH = 1280.0; //pixelzzz
const double PIXEL_HEIGHT = 720.0; //pixelzzz
const double HORIZONTAL_CENTER_LINE = PIXEL_WIDTH / 2.0 - 0.5; // 0.5 offset     //639.5; //pixelzzz
const double VERTICAL_CENTER_LINE = PIXEL_HEIGHT / 2.0 - 0.5;   // 0.5 offset

//CHECK BELOW
const double CAMERA_ANGLE = 45.0; //45.0; //degrees
const double CAMERA_HEIGHT = 15.9; //inches
//const double OFFSET_TO_FRONT = 0.0; //inches

string WINDOW_NAME = "High Goal Window";

// LOWER THE HUE THRESHOLD
const int LOWER_GREEN_HUE = 50;  // 30;
const int LOWER_GREEN_SAT = 100;  // 60
const int LOWER_GREEN_VAL = 30;  // 60

const int UPPER_GREEN_HUE = 80; // 130
const int UPPER_GREEN_SAT = 255; // 255
const int UPPER_GREEN_VAL = 255; // 255

const double CAP_SAT = 0.5;

const int MIN_AREA = 2000; //was 2000
RNG rng(12345);

int main() {
    system("v4l2-ctl -d /dev/video2 -c exposure_auto=1 -c exposure_absolute=5 -c brightness=30");
    cout << "Built with OpenCV B-) " << CV_VERSION << endl;
    Mat image;
    VideoCapture capture(2);
    //  capture.set(CV_CAP_PROP_EXPOSURE, 0.0);
//    capture.set(CV_CAP_PROP_SATURATION, CAP_SAT);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, PIXEL_WIDTH);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, PIXEL_HEIGHT);

    if (!capture.isOpened()) {
        cout << "!!! Failed to open camera darn :((((( " << endl;
        return -1;
    }
    
    Mat frame;
    //Prepare our context and publisher
    context_t context(1);
    socket_t publisher(context, ZMQ_PUB);
    publisher.bind("tcp://*:5564");
    
    for(;;) {
        //READING
        if (!capture.read(frame)) {
            break;
        }

        Mat origImage = frame.clone();
        //imshow(WINDOW_NAME, origImage);
        //STARTING PROCESSING
        medianBlur(frame,frame, 5);

        Mat hsvImage;
        cvtColor(frame, hsvImage, COLOR_BGR2HSV);

        Mat greenRange;
        inRange(hsvImage, Scalar(LOWER_GREEN_HUE, LOWER_GREEN_SAT, LOWER_GREEN_VAL),
                Scalar(UPPER_GREEN_HUE, UPPER_GREEN_SAT, UPPER_GREEN_VAL),
                greenRange);

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
            drawContours(origImage, contours, c, color, 2, 8, hierarchy, 0, Point() );
            approxPolyDP( Mat(contours[c]), contours_poly[c], 3, true);
            boundRect[c] = boundingRect( Mat(contours_poly[c]) );
            //double aspectRatio = (boundRect[c].br().x - boundRect[c].tl().x)/(boundRect[c].br().y - boundRect[c].tl().y);
            //Be careful for aspectRatio
            rectangle(origImage, boundRect[c].tl(), boundRect[c].br(), color, 2, 8, 0);
        }

        for(int c = 0; c < boundRect.size();) {
            //       cout << "bound rect area: " << boundRect[c].area() << endl;
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
        //double centerOfTargets = (boundRect[upRect].tl().x + boundRect[downRect].br().x) / 2.0;
        // ONLY TAKING TOP RECTANGLE
        double centerOfTargets = (boundRect[upRect].tl().x + boundRect[upRect].br().x) / 2.0;
        //FINDING ANGLE
        double angleToMoveApprox;
        angleToMoveApprox = (centerOfTargets - HORIZONTAL_CENTER_LINE) * HORIZONTAL_FOV / PIXEL_WIDTH;
        cout << "approx angle: " << angleToMoveApprox << endl;
        cout << "pixel distance for centers: " << centerOfTargets - HORIZONTAL_CENTER_LINE << endl;

        double angleToMoveAgain;
        angleToMoveAgain = atan((centerOfTargets - HORIZONTAL_CENTER_LINE) / FOCAL_LENGTH);
        angleToMoveAgain = (angleToMoveAgain *180/PI); //converts radients to degrees
        cout << "angleToMoveAgain: " << angleToMoveAgain << endl;
       /*
        double distanceToHighGoal;
	double groundDistance;
        double pixelWidthOfTargets = boundRect[upRect].br().x - boundRect[upRect].tl().x;
        //double y = boundRect[downRect].br().y + (boundRect[upRect].tl().y - boundRect[downRect].br().y) / 2.0;
        //y = -(2 *y / PIXEL_HEIGHT -1);
        //distanceToTopOfGoal = (TOP_TARGET_HEIGHT - CAMERA_HEIGHT) / tan((y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE) * PI / 180);
        // WE MAY HAVE TO ACCOUNT FOR ANGLE OF CAMERA
        // groundDistance = distanceToHighGoal * cos(angleOfCamera)
        distanceToHighGoal = WIDTH_OF_GOAL * FOCAL_LENGTH / pixelWidthOfTargets;
	groundDistance = distanceToHighGoal * cos(CAMERA_ANGLE);
        cout << "distance to high goal: " << distanceToHighGoal << endl;
        cout << "ground distance: " << groundDistance << endl;
        */
/*
	double distanceToHighGoal;
	double groundDistance;
	double pixelHeightDiffOfTopOfTargets = boundRect[upRect].tl().y - boundRect[downRect].tl().y;
	distanceToHighGoal = HEIGHT_DIFF_OF_TOP_OF_TARGETS * FOCAL_LENGTH / pixelHeightDiffOfTopOfTargets;
	groundDistance = distanceToHighGoal * cos(CAMERA_ANGLE);
	cout << "distance to high goal: " << distanceToHighGoal << endl;
	cout << "ground distance: " << groundDistance << endl;
*/
	cout << "boundRect[upRect].tl().y: " << boundRect[upRect].tl().y << endl;
        double angleToTopOfTopTarget = (VERTICAL_CENTER_LINE - boundRect[upRect].tl().y) * VERTICAL_FOV / PIXEL_HEIGHT;
        double heightDiffOfCameraAndTopTarget = HEIGHT_OF_TOP_OF_TOP_TARGET - CAMERA_HEIGHT;
        double groundDistance_Top = heightDiffOfCameraAndTopTarget / tan((CAMERA_ANGLE + angleToTopOfTopTarget) * PI / 180.0);
	cout << "angleToTopOfTopTarget: " << angleToTopOfTopTarget << endl;
	cout << "groundDistance_Top: " << groundDistance_Top << endl;

	double angleToTopOfBottomTarget = (VERTICAL_CENTER_LINE - boundRect[downRect].tl().y) * VERTICAL_FOV / PIXEL_HEIGHT;
	double heightDiffOfCameraAndBottomTarget = HEIGHT_OF_TOP_OF_BOTTOM_TARGET - CAMERA_HEIGHT;
	double groundDistance_Bottom = heightDiffOfCameraAndBottomTarget / tan((CAMERA_ANGLE + angleToTopOfBottomTarget) * PI / 180.0);
	cout << "angleToTopOfBottomTarget: " << angleToTopOfBottomTarget << endl;
	cout << "groundDistance_Bottom: " << groundDistance_Bottom << endl;

	double groundDistance_Average = (groundDistance_Top + groundDistance_Bottom) / 2.0;
        imshow(WINDOW_NAME, origImage);
	cout << "groundDistance_Average: " << groundDistance_Average << endl;

	s_sendmore(publisher, "ANGLE");
	string pub_string_approx = to_string(angleToMoveApprox);
	s_send(publisher, pub_string_approx);

	s_sendmore(publisher, "DISTANCE");
	string another_string = to_string(groundDistance_Average);
	s_send(publisher, another_string);

        char key = cvWaitKey (10);
        if (key == 27) { //Esc
            break;
        }
    }
    return 0;
}
