#pragma once

#include "../parameters.h"
#include "../utility/utility.h"

namespace basic
{
    
class PointFactor : public ceres::SizedCostFunction<2, 7, 7, 3>
{
public:
    Eigen::Vector3d pt_c;
    static Eigen::Matrix2d sqrt_info;
    
    PointFactor() = delete;

    PointFactor(const Eigen::Vector3d& _pt_w);
    
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;


};
}