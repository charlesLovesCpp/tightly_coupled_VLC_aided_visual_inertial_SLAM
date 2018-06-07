#include "point.h"

#include "feature.h"
#include "frame.h"
#include "aprilTag.h"

namespace basic {

int Point::tag_point_counter = 0;

Point::Point(const int& _id, const Vector3d& _P_w) :
    type(TYPE_INIT),
    id(_id), 
    P_w(_P_w),
    n_obs(0)
{}

Point::~Point()
{}

void Point::setPosition(const Vector3d& _P_w)
{
    P_w = _P_w;
    position[0] = P_w.x();
    position[1] = P_w.y();
    position[2] = P_w.z();

    if (type != TYPE_DELETED)
	type = TYPE_TRIANGULATED;
}


void Point::setPositionByPara()
{
    P_w = Vector3d(position[0], position[1], position[2]);
    
    if (type != TYPE_DELETED)
	type = TYPE_TRIANGULATED;
}

void Point::addFrameRef(Feature* _ftr)
{
    obs.push_back(_ftr);
    n_obs++;
}

Feature* Point::findFrameRef(Frame* _frame)
{
    
    auto it = std::find_if(obs.begin(), obs.end(), [_frame](Feature* ftr) {
	return _frame == ftr->frame;
    });
    if (it != obs.end())
	return *it;
    else
	return NULL;
}


bool Point::deleteFrameRef(Frame* _frame)
{
    auto it = find_if(obs.begin(), obs.end(), [_frame](Feature* ftr) {
	return _frame == ftr->frame;
    });
    if (it != obs.end()) {
	obs.erase(it);
	n_obs--;
	if (obs.size() == 0) 
	    type = TYPE_DELETED;
	return true;
    } else
	return false;
}



}
