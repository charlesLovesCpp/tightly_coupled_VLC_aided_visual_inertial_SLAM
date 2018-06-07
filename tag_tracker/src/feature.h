#pragma once

#include "parameters.h"
#include <boost/shared_ptr.hpp>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

namespace basic 
{

class Point;

class Feature
{
public:
    
    enum FeatureType {
	CORNER,
	AprilTag
    };
    
    Point* point_;
    Vector2d uv_;
    Vector3d point_c_;
    double depth_;
    double inv_depth_;
    FeatureType type_;
    
    Feature(const Vector2d& uv) : 
	uv_(uv),
	point_(NULL)
    {}
    
    Feature(const Vector2d& uv, const Vector3d& point_c) : 
	uv_(uv),
	point_c_(point_c),
	point_(NULL)
    {}    
    
    
};

}