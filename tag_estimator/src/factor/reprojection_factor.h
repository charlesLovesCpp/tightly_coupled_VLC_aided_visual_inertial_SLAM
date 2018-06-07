#pragma once

#include "../parameters.h"
#include "../utility/utility.h"

namespace basic
{
    
class ReprojectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>    
{
public:
    Vector3d pts_i;
    Vector3d pts_j;
    static Eigen::Matrix2d sqrt_info;
    
    ReprojectionFactor() = delete;
    
    ReprojectionFactor(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j);
    
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    
};
}