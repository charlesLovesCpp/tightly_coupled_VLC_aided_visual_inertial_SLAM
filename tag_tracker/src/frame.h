#pragma once

#include <list>

#include "parameters.h"

namespace basic
{

class Point;
class Feature;
class AprilTagFeature;

typedef list<Feature> Features;

class Frame
{
public:
    int id_;
    double time_stamp_;
    list<Feature> features_;
    list<AprilTagFeature> apriltag_features_;
    
};
    
}