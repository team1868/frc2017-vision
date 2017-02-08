//#include "opencv2/objdetect.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/videoio.hpp"
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;
using namespace std;

/* IMPORTANT IMPORTANT
   RUN v4l2-ctl -d /dev/video1 -c exposure_auto=1 -c exposure_auto_priority=0 -c exposure_absolute=10
   RUN v4l2-ctl -d /dev/video1 -c white_balance_temperature_auto=0
*/
#define PI 3.14159265

//FIELD SPECS
const int DIST_BETWEEN_GOAL_TARGETS = 7; //inches
const int TOP_TARGET_HEIGHT = 88; //inches, top of tape
const int BOTTOM_TARGET_HEIGHT = 78; //inches, bottom of tape

//CAMERA SPECS
const double DIAGONAL_FOV = 68.5; //degrees
const double HORIZONTAL_FOV = 61.39; //degrees
const double VERTICAL_FOV = 36.0; //degrees
const double FOCAL_LENGTH = 539.0485745586101; //pixelzzz
const int PIXEL_WIDTH = 640; //pixelzzz
const int PIXEL_HEIGHT = 480; //pixelzzz
const double CENTER_LINE = 319.5; //pixelzzz
//CHECK BELOW
const double CAMERA_ANGLE = 10; //degrees
const double CAMERA_HEIGHT = 24; //inches
const double OFFSET_TO_FRONT = 10; //inches

string WINDOW_NAME = "lol";

const int LOWER_GREEN_HUE = 50;
const int LOWER_GREEN_SAT = 60;
const int LOWER_GREEN_VAL = 60;

const int UPPER_GREEN_HUE = 150;
const int UPPER_GREEN_SAT = 255;
const int UPPER_GREEN_VAL = 255;

const int MIN_AREA = 2500;
RNG rng(12345); 

int main()
{
  cout << "Built with OpenCV B-) " << CV_VERSION << endl;
  Mat image;
  VideoCapture capture(1);
  //capture.set(CV_CAP_PROP_EXPOSURE, 0.3);
  capture.set(CV_CAP_PROP_SATURATION, 0.5);
  //capture.set(CV_CAP_PROP_BRIGHTNESS, 0.001); 
  //capture.set(CV_CAP_PROP_CONTRAST, 0.5);

  if (!capture.isOpened())
  {
    cout << "!!! Failed to open camera darn :((((( \n";
    return -1;
  }

  Mat frame;

  for(;;) {
    if (!capture.read(frame)) {
      break;
    }
    Mat origImage = frame.clone();
    medianBlur(frame, frame, 5);

    Mat hsvImage;
    cvtColor(frame, hsvImage, COLOR_BGR2HSV);

    Mat greenRange;
    inRange(hsvImage, Scalar(LOWER_GREEN_HUE, LOWER_GREEN_SAT, LOWER_GREEN_VAL), Scalar(UPPER_GREEN_HUE, UPPER_GREEN_SAT, UPPER_GREEN_VAL), greenRange);

    Mat grayImage;
    cvtColor(frame, grayImage, COLOR_BGR2GRAY);

    GaussianBlur(greenRange, greenRange, cv::Size(9, 9), 2, 2);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(greenRange, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );

    int leftContour;
    int rightContour;

    for(int c = 0; c < contours.size(); c++) {
      Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      if(contourArea(contours[c]) < MIN_AREA) {
        contours.erase(contours.begin() + c);
      }
      drawContours( origImage, contours, c, color, 2, 8, hierarchy, 0, Point() );
      approxPolyDP( Mat(contours[c]), contours_poly[c], 3, true );
      boundRect[c] = boundingRect( Mat(contours_poly[c]) );
      double aspectRatio = (boundRect[c].br().x - boundRect[c].tl().x)/(boundRect[c].br().y - boundRect[c].tl().y);
      if(aspectRatio < 1.1 || aspectRatio > 1.7) {
        continue;
      }
      
      rectangle (origImage, boundRect[c].tl(), boundRect[c].br(), color, 2, 8, 0);
    }
//DO A THING FOR ASPECT RATIO
    if(boundRect[0].tl().x < boundRect[1].tl().x) {
      leftContour = 0;
      rightContour = 1;
    } else {
      leftContour = 1;
      rightContour = 0;
    }
    double centerOfTargets = (boundRect[leftContour].tl().x + boundRect[rightContour].br().x) / 2.0;

//FINDING ANGLE
    double angleToMoveApprox;
    angleToMoveApprox = (centerOfTargets - CENTER_LINE) * HORIZONTAL_FOV / PIXEL_WIDTH;
    cout << "yawwww " << angleToMoveApprox << endl;
    cout << "pixel distance for centers " << centerOfTargets - CENTER_LINE << endl;

    double angleToMoveAgain;
    angleToMoveAgain = atan((centerOfTargets - CENTER_LINE) / FOCAL_LENGTH);
    angleToMoveAgain = angleToMoveAgain * 180 / PI; //CONVERT TO DEGREEZ
    cout << "suh " << angleToMoveAgain << endl;

    Rect rectForCalc;
    if(boundRect[leftContour].height > boundRect[rightContour].height) {
      rectForCalc = boundRect[leftContour];
    } else {
      rectForCalc = boundRect[rightContour];
    }

    double distanceToPeg;
    double y = rectForCalc.br().y + rectForCalc.height / 2.0;
    y = -(2 * y / PIXEL_HEIGHT - 1);
//change top target height to pegs height
    distanceToPeg = (TOP_TARGET_HEIGHT - CAMERA_HEIGHT) / tan(y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE * PI / 180);
    cout << "hallo distanc " << distanceToPeg << endl;

//    imshow(WINDOW_NAME, origImage);
//    imshow("suh", frame);
//    imshow("yooo green", greenRange);
//    imshow("yooo gray", grayImage);
// CHECK FOR RELEASE CAPTURE
    char key = cvWaitKey(10);
    if (key == 27) { // ESC
      break;
    }
  }
  return 0;
}
