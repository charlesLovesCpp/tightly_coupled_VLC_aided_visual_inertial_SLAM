#pragma once

#include <boost/shared_ptr.hpp>

#include "parameters.h"


namespace basic 
{
//class Frame;

class Point 
{
public:
    static int point_counter_;
    int id_;
    Vector3d pos_w_;
    //list<Frame> obs_frames_;
    
    Point(const Vector3d& pos);
    ~Point();
};
    
    
}

