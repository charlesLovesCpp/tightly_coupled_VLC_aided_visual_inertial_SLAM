#pragma once

#include "parameters.h"
#include <boost/shared_ptr.hpp>

namespace basic
{

class Point;
class Feature;
class AprilTagFeature;
class IntegrationBase;

// typedef list<Feature*> Features;

class Frame
{
public:
    static int frame_counter;					// global counter for frames
    int window_counter;						// index in window
    int id;							// unique id for each frame		
    double timestamp;						// time of creation	
    list<Feature*> fts;					
    list<AprilTagFeature*> apriltag_fts;			
    boost::shared_ptr<IntegrationBase> pre_integration;
    Eigen::Vector3d P_w;
    Eigen::Matrix3d R_w_b;
    Eigen::Quaterniond Quat_w_b;
    Eigen::Vector3d V_w;
    Eigen::Vector3d Ba;
    Eigen::Vector3d Bg;
    
    double para_Pose[SIZE_POSE];
    double para_SpeedBias[SIZE_SPEEDBIAS];
 
    Frame(const int& _id = -1, const int& _window_counter = -1, const double& _timestamp = 0.0);
    
    ~Frame();
    
    void clear();
    
    void setFrameId(const int& _id);
    
    void setWindowIndex(const int& _window_counter);
    
    void setTimestamp(const double& _timestamp);
    
    void setPosition(const Vector3d& _P);
    
    void setRotation(const Eigen::Matrix3d& _R);
    
    void setVelocity(const Vector3d& _V);
    
    void setBa(const Vector3d& _Ba);
    
    void setBg(const Vector3d& _Bg);
    
    void setWindowCounter(const int& i);
    
    void resetPRVBaBgByPara();
    
    void addFeature(Feature* _ftr);
    
    void addAprilTag(AprilTagFeature* _apriltag);
    
    void fusePreintegrationInFront(boost::shared_ptr<IntegrationBase> _pre_integration_front);
    


};
    
}