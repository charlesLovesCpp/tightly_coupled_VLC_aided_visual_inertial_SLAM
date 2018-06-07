#pragma once

#include <ros/ros.h>

#include "parameters.h"
#include "tag_detector.h"

class FeatureTracker 
{
public:
    TagDetector tagDetector;
    
    
    FeatureTracker();
    
    void readImage(cv::Mat&  _img, double _cur_time);
};