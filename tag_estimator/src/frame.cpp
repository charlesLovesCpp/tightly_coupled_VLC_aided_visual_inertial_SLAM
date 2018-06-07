#include "frame.h"

#include "point.h"
#include "feature.h"
#include "aprilTag.h"
#include "factor/integration_base.h"

namespace basic 
{
int Frame::frame_counter = 0;

Frame::Frame(const int& _id, const int& _window_counter, const double& _timestamp):// id is frame_counter
    id(_id),
    window_counter(_window_counter),
    timestamp(_timestamp)
{
    R_w_b.setIdentity();
    Quat_w_b = R_w_b;
    P_w.setZero();
    V_w.setZero();
    Ba.setZero();
    Bg.setZero();
 
    para_Pose[0] = P_w.x();
    para_Pose[1] = P_w.y();
    para_Pose[2] = P_w.z();
    para_Pose[3] = Quat_w_b.x();
    para_Pose[4] = Quat_w_b.y();
    para_Pose[5] = Quat_w_b.z();
    para_Pose[6] = Quat_w_b.w();

    para_SpeedBias[0] = V_w.x();
    para_SpeedBias[1] = V_w.y();
    para_SpeedBias[2] = V_w.z();   
    para_SpeedBias[3] = Ba.x();
    para_SpeedBias[4] = Ba.y();
    para_SpeedBias[5] = Ba.z();  
    para_SpeedBias[6] = Bg.x();
    para_SpeedBias[7] = Bg.y();
    para_SpeedBias[8] = Bg.z();
    
    
}


Frame::~Frame()
{}

void Frame::setFrameId(const int& _id)
{
    id = _id;
}

void Frame::setWindowIndex(const int& _window_counter)
{
    window_counter = _window_counter;
}

void Frame::setTimestamp(const double& _timestamp)
{
    timestamp = _timestamp;
}

void Frame::setPosition(const Vector3d& _P)
{
    P_w = _P;
    
    para_Pose[0] = P_w.x();
    para_Pose[1] = P_w.y();
    para_Pose[2] = P_w.z();
}

void Frame::setRotation(const Matrix3d& _R)
{
    R_w_b = _R;
    Quat_w_b = _R;
    
    para_Pose[3] = Quat_w_b.x();
    para_Pose[4] = Quat_w_b.y();
    para_Pose[5] = Quat_w_b.z();
    para_Pose[6] = Quat_w_b.w();
}


void Frame::setVelocity(const Vector3d& _V)
{
    V_w = _V;
    
    para_SpeedBias[0] = V_w.x();
    para_SpeedBias[1] = V_w.y();
    para_SpeedBias[2] = V_w.z();
    
}

void Frame::setBa(const Vector3d& _Ba)
{
    Ba = _Ba;
    
    para_SpeedBias[3] = Ba.x();
    para_SpeedBias[4] = Ba.y();
    para_SpeedBias[5] = Ba.z();
}

void Frame::setBg(const Vector3d& _Bg)
{
    Bg = _Bg;
    
    para_SpeedBias[6] = Bg.x();
    para_SpeedBias[7] = Bg.y();
    para_SpeedBias[8] = Bg.z();
}


void Frame::setWindowCounter(const int& i)
{
    window_counter = i;
}

void Frame::resetPRVBaBgByPara()
{
    P_w = Vector3d(para_Pose[0], para_Pose[1], para_Pose[2]);
    Quat_w_b = Eigen::Quaterniond(para_Pose[6], para_Pose[3], para_Pose[4], para_Pose[5]);  
    R_w_b = Quat_w_b.toRotationMatrix();
    V_w = Vector3d(para_SpeedBias[0], para_SpeedBias[1], para_SpeedBias[2]);
    Ba = Vector3d(para_SpeedBias[3], para_SpeedBias[4], para_SpeedBias[5]);
    Bg = Vector3d(para_SpeedBias[6], para_SpeedBias[7], para_SpeedBias[8]);
}

void Frame::fusePreintegrationInFront(boost::shared_ptr<IntegrationBase> _pre_integration_front)
{
    pre_integration.swap(_pre_integration_front);
    
    for (unsigned int i = 0; i < _pre_integration_front->dt_buf.size(); ++i) {
	pre_integration->push_back(_pre_integration_front->dt_buf[i], 
				   _pre_integration_front->acc_buf[i],
				   _pre_integration_front->gyr_buf[i]);
	pre_integration->dt_buf.push_back(_pre_integration_front->dt_buf[i]);
	pre_integration->acc_buf.push_back(_pre_integration_front->acc_buf[i]);
	pre_integration->gyr_buf.push_back(_pre_integration_front->gyr_buf[i]);
    }
}

void Frame::addFeature(Feature* _ftr)
{
    fts.push_back(_ftr);
}

void Frame::addAprilTag(AprilTagFeature* _apriltag)
{
    apriltag_fts.push_back(_apriltag);
}

void Frame::clear()
{
    for (auto& ftr : fts) {
	Point* pt = ftr->point;
	if(!(pt->deleteFrameRef(ftr->frame)))
	    ROS_WARN("Feature deletion failure");
	delete ftr;
    }
    fts.clear();
    
    for (auto& tag_ftr : apriltag_fts) {
	AprilTagInstance* tag_instance = tag_ftr->tag_instance;
	if(!(tag_instance->deleteFrameRef(tag_ftr->frame)))
	    ROS_WARN("AprilTag deletion failure");
	delete tag_ftr;
    }
    apriltag_fts.clear();
    
    pre_integration.reset();
}


}