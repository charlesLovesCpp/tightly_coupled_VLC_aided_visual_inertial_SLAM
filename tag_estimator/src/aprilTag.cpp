#include "aprilTag.h"

#include "frame.h"
#include "feature.h"
#include "point.h"

namespace basic 
{

AprilTagFeature::AprilTagFeature(Frame* _frame, const std::vector< Feature* >& _tag_fts, const double& _timestamp):
    frame(_frame),
    tag_fts(_tag_fts),
    timestamp(_timestamp)
{
    tag_fts[0]->pt_c;
    tag_fts[2]->pt_c;
    double x = (tag_fts[0]->pt_c.x() + tag_fts[2]->pt_c.x()) / 2;
    double y = (tag_fts[0]->pt_c.y() + tag_fts[2]->pt_c.y()) / 2;
    ftr_center = Vector3d(x, y, 1);
}

AprilTagFeature::~AprilTagFeature()
{}    

void AprilTagFeature::setAprilTag(AprilTagInstance* _tag_instance)
{
    tag_instance = _tag_instance;
}


AprilTagInstance::AprilTagInstance(const int& _id, const double& _size_w, const Matrix3d& _R_w_t, const Vector3d& _t_w):
    type(TYPE_GOOD),
    id(_id),
    size_w(_size_w),
    R_w_t(_R_w_t),
    t_w(_t_w),
    n_obs(0)
{
    // Tag frame
    double s = _size_w / 2;

    pts_t.push_back(Vector3d(-s, -s, 0));
    pts_t.push_back(Vector3d( s, -s, 0));
    pts_t.push_back(Vector3d( s,  s, 0));
    pts_t.push_back(Vector3d(-s,  s, 0));
    
    for (unsigned int i = 0; i < 4; ++i) {
	Vector3d tmp_pt_w = R_w_t * pts_t[i] + t_w;
	pts_w.push_back(tmp_pt_w);
	
	position[i][0] = tmp_pt_w.x();
	position[i][1] = tmp_pt_w.y();
	position[i][2] = tmp_pt_w.z();
    }
    
    pt_center = t_w;
    position_center[0] = t_w.x();
    position_center[1] = t_w.y();
    position_center[2] = t_w.z();
}


AprilTagInstance::~AprilTagInstance()
{}

void AprilTagInstance::addFrameRef(AprilTagFeature* _tag_ftr)
{
    tag_obs.push_back(_tag_ftr);
    n_obs++;
}

AprilTagFeature* AprilTagInstance::findFrameRef(Frame* _frame)
{
    auto it = std::find_if(tag_obs.begin(), tag_obs.end(), [_frame](AprilTagFeature* tag_ftr) {
	return _frame == tag_ftr->frame;
    });
    if (it != tag_obs.end())
	return *it;
    else
	return NULL;
}


bool AprilTagInstance::deleteFrameRef(Frame* _frame)
{
    auto it = find_if(tag_obs.begin(), tag_obs.end(), [_frame](AprilTagFeature* tag_ftr) {
	return tag_ftr->frame == _frame;
    });
    if (it != tag_obs.end()) {
	tag_obs.erase(it);
	if (tag_obs.size() == 0) {
	    type = TYPE_DELETED;
	}
	n_obs--;
	return true;
    } else
	return false;
}


}
