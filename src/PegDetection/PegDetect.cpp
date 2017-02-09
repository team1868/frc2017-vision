//#include "opencv2/imgproc.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/videoio.hpp"
#include "zhelpers.hpp"
#include <zmq.hpp>
#include <string>
#include <chrono>
#include <thread>
#include <string>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;
using namespace zmq;
using namespace std;

/* IMPORTANT IMPORTANT
   RUN v4l2-ctl -d /dev/video1 -c exposure_auto=1 -c exposure_absolute=10
   RUN v4l2-ctl -d /dev/video1 -c white_balance_temperature_auto=0
*/
#define PI 3.14159265

//FIELD SPECS
const double DIST_BETWEEN_BOILER_TARGETS = 10; //inches, vertical
const double TOP_BOILER_TARGET_HEIGHT = 88; //inches, top of tape
const double BOTTOM_BOILER_TARGET_HEIGHT = 78; //inches, bottom of tape

const double DIST_BETWEEN_PEG_TARGETS = 10.25; //inches, horizontal, outside to outside
const double TOP_PEG_TARGET_HEIGHT = 15.75; //inches, top of tape
const double BOTTOM_PEG_TARGET_HEIGHT = 10.75; //inches, bottom of tape

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
  capture.set(CV_CAP_PROP_SATURATION, 0.5);
  cout << "gr fps " << capture.get(CV_CAP_PROP_FPS);

  if (!capture.isOpened())
  {
    std::cout << "!!! Failed to open camera darn :((((( \n";
    return -1;
  }

  Mat frame;
  //  Prepare our context and publisher
  context_t context(1);
  socket_t publisher(context, ZMQ_PUB);

  // cout << publisher.setsockopt(ZMQ_SNDHWM, 1) << endl;
  // cout << publisher.setsockopt(ZMQ_SNDHWM, 1) << endl;
  //int rc = zmq_setsockopt (publisher, ZMQ_SNDHWM, 1);
  //assert (rc == 0);
  publisher.bind("tcp://*:5563");

  for(;;) {
    if (!capture.read(frame)) {
      break;
    }
    Mat origImage = frame.clone();
    medianBlur(frame, frame, 5);

    Mat hsvImage;
    cvtColor(frame, hsvImage, COLOR_BGR2HSV);

    Mat greenRange;
    inRange(hsvImage, Scalar(LOWER_GREEN_HUE, LOWER_GREEN_SAT, LOWER_GREEN_VAL),
                      Scalar(UPPER_GREEN_HUE, UPPER_GREEN_SAT, UPPER_GREEN_VAL),
                      greenRange);

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
        continue;
      }
      drawContours( origImage, contours, c, color, 2, 8, hierarchy, 0, Point() );
      approxPolyDP( Mat(contours[c]), contours_poly[c], 3, true );
      boundRect[c] = boundingRect( Mat(contours_poly[c]) );

      rectangle (origImage, boundRect[c].tl(), boundRect[c].br(), color, 2, 8, 0);
    }

    for(int c = 0; c < boundRect.size();) {
      if(boundRect[c].area() < MIN_AREA) {
        boundRect.erase(boundRect.begin()+c);
        continue;
      }
      c++;
    }

    if(boundRect.size() < 2) {
      cout << "no rectangles :(" << endl;
      continue;
    }

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

//    double angleToMoveAgain;
//    angleToMoveAgain = atan((centerOfTargets - CENTER_LINE) / FOCAL_LENGTH);
//    angleToMoveAgain = angleToMoveAgain * 180 / PI; //CONVERT TO DEGREEZ
//    cout << "suh " << angleToMoveAgain << endl;

    Rect rectForCalc;
    if(boundRect[leftContour].height > boundRect[rightContour].height) {
      rectForCalc = boundRect[leftContour];
    } else {
      rectForCalc = boundRect[rightContour];
    }

    double distanceToPeg;
    double y = rectForCalc.br().y + rectForCalc.height / 2.0;
    y = -(2 * y / PIXEL_HEIGHT - 1);
    distanceToPeg = (TOP_PEG_TARGET_HEIGHT - CAMERA_HEIGHT) / tan(y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE * PI / 180);
    cout << "hallo distanc " << distanceToPeg << endl;

    imshow(WINDOW_NAME, origImage);
    //imshow("suh", frame);
    //imshow("yooo green", greenRange);
//    imshow("yooo gray", grayImage);

    //  Write two messages, each with an envelope and content
    s_sendmore (publisher, "A");
    s_send (publisher, "We don't want to see this");
    s_sendmore (publisher, "B");
       string pub_string_approx = to_string(angleToMoveApprox);
    s_send (publisher, pub_string_approx);
    //s_send (publisher, i);
//    this_thread::sleep_for(chrono::milliseconds(1));

    char key = cvWaitKey(10);
    if (key == 27) { // ESC
      break;
    }
  }
  return 0;
}
