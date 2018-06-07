#pragma once

#include "../utility/utility.h"
#include "../parameters.h"


namespace basic
{

class IntegrationBase 
{
public:
    // Records for mid-point integration
    double dt;
    Vector3d acc_0, gyr_0;
    Vector3d acc_1, gyr_1;
    
    // These values are assigned at the beginning.
    Vector3d linearized_acc, linearized_gyr;
    Vector3d linearized_ba, linearized_bg;
    
    Eigen::Matrix<double, 15, 15> jacobian;
    Eigen::Matrix<double, 15, 15> covariance;
    Eigen::Matrix<double, 18, 18> noise;
    
    // Result of pre-integration
    double sum_dt;
    Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Vector3d delta_v;
    
    // Data
    vector<double> dt_buf;
    vector<Vector3d> acc_buf;
    vector<Vector3d> gyr_buf;
    
    
    IntegrationBase() = delete;
    IntegrationBase(const Vector3d& _acc_0, const Vector3d& _gyr_0, const Vector3d& _ba, const Vector3d& _bg) :
	acc_0(_acc_0),
	gyr_0(_gyr_0),
	linearized_ba(_ba),
	linearized_bg(_bg),
	sum_dt(0.0),
	jacobian(Eigen::Matrix<double, 15, 15>::Identity()),
	covariance(Eigen::Matrix<double, 15, 15>::Identity()),
	delta_p(Vector3d::Zero()),
	delta_q(Eigen::Quaterniond::Identity()),
	delta_v(Vector3d::Zero())
    {
	noise = Eigen::Matrix<double, 18, 18>::Zero();
        noise.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();//P
        noise.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();//R
        noise.block<3, 3>(6, 6) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();//V
        noise.block<3, 3>(9, 9) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();//R_V
        noise.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();//Ba
        noise.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();//Bg
    };
    
    Eigen::Matrix3d getSymmetricMatrix(const Vector3d& v) {
	Eigen::Matrix3d M;
	M << 0, -v(2), v(1),
	    v(2), 0, -v(0),
	    -v(1), v(0), 0;
	return M;
    };
    
    
    void midPointIntegration(const double& _dt, 
			const Vector3d& _acc_0, const Vector3d& _gyr_0,
			const Vector3d& _acc_1, const Vector3d& _gyr_1,
			const Vector3d& delta_p, const Eigen::Quaterniond& delta_q, const Vector3d& delta_v,
			const Vector3d& linearized_ba, const Vector3d& linearized_bg,
			Vector3d& result_delta_p, Eigen::Quaterniond& result_delta_q, Vector3d& result_delta_v,
			Vector3d& result_linearized_ba, Vector3d& result_linearized_bg, bool update_jacobian) 
    {
	Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
	Vector3d un_gyr_0 = 0.5 * (_gyr_0 + gyr_1) - linearized_bg;
	result_delta_q = delta_q * Quaterniond(1, un_gyr_0(0) * _dt / 2, un_gyr_0(1) * _dt / 2, un_gyr_0(2) * _dt / 2);
	Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
	Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
	result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
	result_delta_v = delta_v + un_acc * _dt;
	result_linearized_ba = linearized_ba;
	result_linearized_bg = linearized_bg;
	
	if (update_jacobian) {
	    Vector3d w_x = un_gyr_0;
	    Vector3d a_0_x = _acc_0 - linearized_ba;
	    Vector3d a_1_x = _acc_1 - linearized_ba;
	    Eigen::Matrix3d R_w_x, R_a_0_x, R_a_1_x;
	    
	    R_w_x = getSymmetricMatrix(w_x);
	    R_a_0_x = getSymmetricMatrix(a_0_x);
	    R_a_1_x = getSymmetricMatrix(a_1_x);
	    
	    MatrixXd F = MatrixXd::Zero(15, 15);
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt + 
                                  -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt + 
                                  -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6) = Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F.block<3, 3>(9, 9) = Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Matrix3d::Identity();
            //cout<<"A"<<endl<<A<<endl;

	    // V is some kind of approximation. 
            MatrixXd V = MatrixXd::Zero(15,18);
	    // jacobian of 
            V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
	    // jacobian of theta the rotation with respect to theta_i the noise of gyro measurement
            V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
	    // jacobian with
            V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);	
	    // jacobian of f_ba, f_bg with respect to the noise relative values
            V.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;

            //step_jacobian = F;
            //step_V = V;
	    
            jacobian = F * jacobian;
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();
	    
	}
    };
    
    void propagate(const double& _dt, const Vector3d& _acc, const Vector3d& _gyr)
    {
	dt = _dt;
	acc_1 = _acc;
	gyr_1 = _gyr;
	
	Vector3d result_delta_p;
        Quaterniond result_delta_q;
        Vector3d result_delta_v;
        Vector3d result_linearized_ba;
        Vector3d result_linearized_bg;

        midPointIntegration(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v,
                            result_linearized_ba, result_linearized_bg, 1);
	
        delta_p = result_delta_p;
        delta_q = result_delta_q;
        delta_v = result_delta_v;
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        delta_q.normalize();
        sum_dt += dt;
        acc_0 = acc_1;
        gyr_0 = gyr_1;  	
    };
    
    // Repropagate result by new ba and bg
    void repropagate(const Vector3d& _linearized_ba, const Vector3d& _linearized_bg) 
    {
	sum_dt = 0.0;
	acc_0 = linearized_acc;
	gyr_0 = linearized_gyr;
	delta_p.setZero();
	delta_q.setIdentity();
	delta_v.setZero();
	linearized_ba = _linearized_ba;
	linearized_bg = _linearized_bg;
	jacobian.setIdentity();
	covariance.setIdentity();
	for (unsigned int i = 0; i < dt_buf.size(); ++i)
	    propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
    };
    
    void push_back(const double& _dt, const Vector3d& _acc, const Vector3d& _gyr)
    {
	dt_buf.push_back(dt);
	acc_buf.push_back(_acc);
	gyr_buf.push_back(_gyr);
	propagate(_dt, _acc, _gyr);
    };
    
    Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
	Eigen::Matrix<double, 15, 1> residuals;
	
        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;

        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
        Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

        residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * g * sum_dt * sum_dt + SCALE_TO_METRIC * (Pj - Pi - Vi * sum_dt)) - corrected_delta_p;// G gravity, Pj - Pi, j>i, Q->R_w_bk
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();// corrected_delta_q's position is different from the equation 24
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (g * sum_dt + SCALE_TO_METRIC* (Vj - Vi)) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;	
	
	return residuals;
    }
    
};

}

