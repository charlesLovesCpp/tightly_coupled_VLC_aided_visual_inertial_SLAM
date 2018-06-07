#pragma once

#include "parameters.h"

namespace basic 
{
    
class Point;
class Frame;
class Feature;
class AprilTagInstance;

class AprilTagFeature 
{
public:  
    Vector3d ftr_center;
    vector<Feature*> tag_fts;
    AprilTagInstance* tag_instance;
    Frame* frame;
    double timestamp;

    AprilTagFeature(Frame* _frame, const vector<Feature*>& _tag_fts, const double& _timestamp);
  
    ~AprilTagFeature();
    
    void setAprilTag(AprilTagInstance* _tag_instance);
}; 

class AprilTagInstance
{
public:
    
    enum TagType {
	TYPE_DELETED,
	TYPE_GOOD
    };
    
    TagType type;
    int id;
    double size_w;

    Vector3d pt_center;
    
    // Corner points in F_t, R_w_t * F_t + t_w = F_w
    vector<Vector3d> pts_t;
    
    // Corner points in F_w
    vector<Vector3d> pts_w;
    
    double position[4][3];
    
    double position_center[3];
    
    // Tag frame -> Wolrd frame
    Vector3d t_w;
    
    Eigen::Matrix3d R_w_t;
    
    list<AprilTagFeature*> tag_obs;
    
    int n_obs;
    
    AprilTagInstance(const int& _id, const double& _size_w, const Eigen::Matrix3d& _R_w_t, const Vector3d& _t_w);
    
    ~AprilTagInstance();
    
    void addFrameRef(AprilTagFeature* _tag_ftr);
    
    AprilTagFeature* findFrameRef(Frame* _frame);
    
    bool deleteFrameRef(Frame* _frame);
    
    void getR();
    
    void toTagFrame();
};



}
