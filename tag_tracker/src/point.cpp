#include "point.h"
#include "feature.h"

namespace basic {

int Point::point_counter_ = 0;

Point::Point(const Vector3d& pos) :
    id_(point_counter_), pos_w_(pos)
{}

    
}
