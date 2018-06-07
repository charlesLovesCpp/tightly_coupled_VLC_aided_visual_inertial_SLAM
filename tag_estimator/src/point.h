#pragma once

#include <algorithm>
#include <numeric>

#include "parameters.h"

namespace basic 
{
    
class Frame;
class Feature;

class Point 
{
public:
    
    // No observation: TYPE_DELETED
    enum PointType {
	TYPE_DELETED,
	TYPE_INIT,
	TYPE_TRIANGULATED
    };
  
    PointType type;
    static int tag_point_counter;
    int id;  
    list<Feature*> obs;
    size_t n_obs;
    Vector3d P_w;
    double position[3];				// ceres Bundle Adjustment variables
    
    
    Point(const int& _id, const Vector3d& _P_w = Vector3d(0, 0, 0));
    
    ~Point();
    
    void setPosition(const Vector3d& _P_w);
    
    void setPositionByPara();
    
    void setParaByPosition();
    
    void addFrameRef(Feature* _ftr);
    
    bool deleteFrameRef(Frame* _frame);
    
    Feature* findFrameRef(Frame* _frame);
};
    
    
}

