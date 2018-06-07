#include "point_factor.h"

namespace basic 
{
Eigen::Matrix2d PointFactor::sqrt_info;

PointFactor::PointFactor(const Vector3d& _pt_c):
    pt_c(_pt_c)
{}


/*
bool PointFactor::Evaluate(const double*const* parameters, double* residuals, double** jacobians) const
{
    Eigen::Vector3d P_w(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Q_w_c(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);    
    
    Eigen::Vector3d Pt_w(parameters[1][0], parameters[1][1], parameters[1][2]); 
    
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    
    Eigen::Matrix3d R_c_w = Q_w_c.inverse().toRotationMatrix();
    Eigen::Vector3d t_c_w = -R_c_w * P_w;

    Eigen::Vector3d pt_c_hat = R_c_w * (Pt_w -  P_w);
    double dep = pt_c_hat.z();

    residual = (pt_c_hat / dep).head<2>() - pt_c.head<2>();
    //residual = sqrt_info * residual;	
    
//     ROS_INFO_STREAM("Point factor: true pt_c: " << pt_c.transpose() << std::endl << 
// 	"------------------------------------------------  estimated pt_c: " << _pt_c.transpose());
	    
    if (jacobians) {
	Eigen::Matrix<double, 2, 3> dr_dptc;
	dr_dptc << 1. / dep, 0, -pt_c_hat(0) / (dep * dep),
		   0, 1. / dep, -pt_c_hat(1) / (dep * dep);	
	// Pi
	if (jacobians[0]) {
	    Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_w(jacobians[0]);
	    Eigen::Matrix<double, 3, 6> jaco_w;
	    
	    jaco_w.leftCols<3>() = -R_c_w;
	    jaco_w.rightCols<3>() = Utility::skewSymmetric(pt_c_hat);
	    
	    jacobian_pose_w.leftCols<6>() = dr_dptc * jaco_w;
	    jacobian_pose_w.rightCols<1>().setZero();	    
	}
	// Pt_w
	if (jacobians[1]) {
	    Eigen::Map<Eigen::Matrix<double, 2, 3>> jacobian_point_w(jacobians[2]);
	    
	    Eigen::Matrix3d jaco_pt = R_c_w;
	    jacobian_point_w = dr_dptc * jaco_pt;
	}
    }
    
    return true;
}*/



bool PointFactor::Evaluate(const double*const* parameters, double* residuals, double** jacobians) const
{
    Eigen::Vector3d P_w(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Q_w_b(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);    
   
    Eigen::Vector3d t_b(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond q_b_c(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    
    Eigen::Vector3d Pt_w(parameters[2][0], parameters[2][1], parameters[2][2]); 
    
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    
    Eigen::Matrix3d R_b_w = Q_w_b.inverse().toRotationMatrix();
    Eigen::Vector3d t_b_w = -R_b_w * P_w;
    Eigen::Matrix3d r_c_b = q_b_c.inverse().toRotationMatrix();
    
    Eigen::Vector3d pt_b = R_b_w * Pt_w + t_b_w;
    Eigen::Vector3d pt_c_s = r_c_b * (pt_b - t_b);
    double dep = pt_c_s.z();
    Eigen::Vector3d _pt_c= pt_c_s / dep;    

    residual = _pt_c.head<2>() - pt_c.head<2>();
    
    residual = sqrt_info * residual;	
        
    if (jacobians) {
	Eigen::Matrix<double, 2, 3> dr_dptc;
	dr_dptc << 1. / dep, 0, -pt_c_s(0) / (dep * dep),
		0, 1. / dep, -pt_c_s(1) / (dep * dep);	
	// Pi
	if (jacobians[0]) {
	    Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_w(jacobians[0]);
	    Eigen::Matrix<double, 3, 6> jaco_w;
	    
	    jaco_w.leftCols<3>() = -r_c_b * R_b_w;
	    jaco_w.rightCols<3>() = r_c_b * Utility::skewSymmetric(pt_b);
	    
	    jacobian_pose_w.leftCols<6>() = dr_dptc * jaco_w;
	    jacobian_pose_w.rightCols<1>().setZero();	    
	}
	// P_ex
	if (jacobians[1]) {
	    Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_ex(jacobians[1]);
	    Eigen::Matrix<double, 3, 6> jaco_ex;
	    
	    jaco_ex.leftCols<3>() = -r_c_b;
	    jaco_ex.rightCols<3>() = Utility::skewSymmetric(pt_c_s);
	    
	    jacobian_pose_ex.leftCols<6>() = dr_dptc * jaco_ex;
	    jacobian_pose_ex.rightCols<1>().setZero();	
	}
	// Pt_w
	if (jacobians[2]) {
	    Eigen::Map<Eigen::Matrix<double, 2, 3>> jacobian_point_w(jacobians[2]);
	    
	    Eigen::Matrix3d jaco_pt = r_c_b * R_b_w;
	    jacobian_point_w = dr_dptc * jaco_pt;
	}
    }
    
    return true;
}

    
}