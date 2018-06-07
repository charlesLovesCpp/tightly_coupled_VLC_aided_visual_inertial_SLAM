#pragma once

#include "../utility/utility.h"
#include "../parameters.h"

namespace basic
{
    
struct ReprojectionError3D
{
    ReprojectionError3D(double _observed_u, double _observed_v):
	observed_u(_observed_u),
	observed_v(_observed_v)
	{}
    
    // compute residuals
    template<typename T>
    bool operator()(const T* const camera_R, const T* const camera_T, const T* point, T* residuals) const {
	T p[3];
	ceres::QuaternionRotatePoint(camera_R, point, p);		// R_c_w * P_w
	p[0] += camera_T[0]; p[1] += camera_T[1]; p[2] += camera_T[2];	// P_c + t_c
	T xp = p[0] / p[2];
	T yp = p[1] / p[2];
	residuals[0] = xp - T(observed_u);
	residuals[1] = yp - T(observed_v);
	return true;
    }
    
    static ceres::CostFunction* Create(const double observed_x,
				       const double observed_y) 
    {
	// <residuals, Quat, t, point3d>
	return (new ceres::AutoDiffCostFunction<ReprojectionError3D, 2, 4, 3, 3>(
		new ReprojectionError3D(observed_x, observed_y)
	));
    }
    
    double observed_u;
    double observed_v;
};
   
}