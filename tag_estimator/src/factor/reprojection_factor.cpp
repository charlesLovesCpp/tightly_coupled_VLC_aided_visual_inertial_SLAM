#include "reprojection_factor.h"

namespace basic 
{

Eigen::Matrix2d ReprojectionFactor::sqrt_info;
    
ReprojectionFactor::ReprojectionFactor(const Vector3d& _pts_i, const Vector3d& _pts_j):
    pts_i(_pts_i),
    pts_j(_pts_j)
{}

bool ReprojectionFactor::Evaluate(const double*const* parameters, double* residuals, double** jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);    
   
    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    
    Eigen::Vector3d t_b(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond q_b_c(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    
    double inv_dep_i = parameters[3][0];
    
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    
    Eigen::Vector3d pt_c_i_s = pts_i / inv_dep_i;
    Eigen::Vector3d pt_b_i = q_b_c * pt_c_i_s + t_b;
    Eigen::Vector3d pt_w_i = Qi * pt_b_i + Pi;
    Eigen::Vector3d pt_b_j = Qj.inverse() * (pt_w_i - Pj);
    Eigen::Vector3d pt_c_j_s = q_b_c.inverse() * (pt_b_j - t_b);
    double dep_j = pt_c_j_s.z();
    Eigen::Vector3d pt_c_j = pt_c_j_s / dep_j;
    
    residual = (pt_c_j - pts_j).head<2>();
    
    residual = sqrt_info * residual;	
    
    if (jacobians) {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d r_b_c = q_b_c.toRotationMatrix();
	Eigen::Matrix<double, 2, 3> dr_dptc;
	dr_dptc << 1. / dep_j, 0, -pt_c_j_s(0) / (dep_j * dep_j),
		   0, 1. / dep_j, -pt_c_j_s(1) / (dep_j * dep_j);
		   
	dr_dptc = sqrt_info * dr_dptc;
		   
	// Pi
	if (jacobians[0]) {
	    Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
	    
	    Eigen::Matrix<double, 3, 6> jaco_i;
	    jaco_i.leftCols<3>() = r_b_c.inverse() * Qj.inverse();
	    jaco_i.rightCols<3>() = - r_b_c.inverse() * Qj.inverse() * Utility::skewSymmetric(pt_b_i);
	    
	    jacobian_pose_i.leftCols<6>() = dr_dptc * jaco_i;
	    jacobian_pose_i.rightCols<1>().setZero();
	}	
	// Pj
	if (jacobians[1]) {
	    Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
	    
	    Eigen::Matrix<double, 3, 6> jaco_j;
	    jaco_j.leftCols<3>() =  - r_b_c.transpose() * Rj.transpose();
	    jaco_j.rightCols<3>() = r_b_c.transpose() * Utility::skewSymmetric(pt_b_j);
	    
	    jacobian_pose_j.leftCols<6>() = dr_dptc * jaco_j;
	    jacobian_pose_j.rightCols<1>().setZero();
	    
	}	
	// P_extrinsic
	if (jacobians[2]) {
	    Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
	    
	    jaco_ex.leftCols<3>() = r_b_c.transpose() * (Rj.transpose() * Ri - Matrix3d::Identity());
	    Eigen::Matrix3d tmp_R = r_b_c.transpose() * Rj.transpose() * Ri * r_b_c;
	    jaco_ex.rightCols<3>() = -tmp_R * Utility::skewSymmetric(pt_c_i_s) + 
				    Utility::skewSymmetric(tmp_R * pt_c_i_s);
            jacobian_ex_pose.leftCols<6>() = dr_dptc * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();	    
	}	
	// inv_dep_i
	if (jacobians[3]) {
	    Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
	    jacobian_feature = dr_dptc * r_b_c.transpose() * Rj.transpose() * Ri * r_b_c * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
	}
    }
    return true;
}


}