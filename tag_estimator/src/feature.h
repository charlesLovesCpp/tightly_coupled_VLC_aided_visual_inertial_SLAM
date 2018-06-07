#pragma once

#include "parameters.h"
#include "frame.h"

#include <boost/shared_ptr.hpp>

namespace basic 
{

class Point;

class Feature
{
public:
    
    Point* point;
    Frame* frame;
    Vector2d px;
    Vector3d pt_c;
    Vector2d velocity;
    double depth;
    double inv_depth;
    double timestamp;
    FeatureType type;
    
    Feature(Frame* _frame, const Vector2d& _px, const Vector3d& _pt_c, const Vector2d& _velocity, const double& _timestamp, const FeatureType& _type) : 
	point(NULL),
	frame(_frame),
	px(_px),
	pt_c(_pt_c),
	velocity(_velocity),
	depth(-1),
	inv_depth(-1),
	timestamp(_timestamp),
	type(_type)
    {}    
    
    Feature(Frame* _frame, const Eigen::Matrix<double, 7, 1>& _ftr, const double& _timestamp, const FeatureType& _type) : 
	point(NULL),
	frame(_frame),
	depth(-1),
	inv_depth(-1),
	timestamp(_timestamp),
	type(_type)
    {
	pt_c.x() = _ftr(0);
	pt_c.y() = _ftr(1);
	pt_c.z() = _ftr(2);
	px.x() = _ftr(3);
	px.y() = _ftr(4);
	velocity.x() = _ftr(5);
	velocity.y() = _ftr(6);
    } 
    
    void setPoint(Point* _point) 
    {
	point = _point;
    }
     
    void setFrame(Frame* _frame)
    {
	frame = _frame;
    }
};

}