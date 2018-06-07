#pragma once

#include <ros/ros.h>

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <list>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>

#include "utility/utility.h"
#include "utility/tic_toc.h"

#define USE_LED_MODE 1

using namespace Eigen;
using namespace std;

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 5;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;
extern Eigen::Vector3d g;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string APRIL_VIO_OUTPUT;
extern std::string IMU_TOPIC;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;

extern double TAG_SIZE;
extern double SCALE_TO_METRIC;

extern map<int, pair<double, pair<Vector3d, Eigen::Matrix3d>>> GLOBAL_TAGS_DIC;

void readParameters(ros::NodeHandle &n);

enum FeatureType
{
    POINT = 0,
    TAG = 1
};


enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
