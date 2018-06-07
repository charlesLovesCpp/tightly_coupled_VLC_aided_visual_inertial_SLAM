#pragma once
#include <ros/ros.h>
#include <ros/console.h>

#include <math.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "tic_toc.h"

using namespace Eigen;
using namespace std;
using namespace camodocal;

enum FeatureType {
    POINT = 0,
    TAG = 1
};


extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;


extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

void readParameters(ros::NodeHandle &n);
