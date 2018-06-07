/*

stringstream ss;
ss << det->id;
string text = ss.str();// tag_text


int feature_id = id_pts.first;
auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it) {
            return it.feature_id == feature_id;
          });
if (it == feature.end())
{}


cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

pnp_succ = cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1);
if(!pnp_succ)
{
    return false;
}


Eigen::Vector3d ng1 = g.normalized();
Eigen::Vector3d ng2{0, 0, 1.0};
R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();


for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
{
    frame_j = next(frame_i);
    MatrixXd tmp_A(3, 3);
    tmp_A.setZero();
    VectorXd tmp_b(3);
    tmp_b.setZero();
    Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
    tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
    tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
    A += tmp_A.transpose() * tmp_A;
    b += tmp_A.transpose() * tmp_b;

}


tmp_A.setZero();
Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
cov_inv.setIdentity();

// 归一化
g.normalized()

G.norm()//sqrt(G(0)^2 + G(1)^2 + G(2)^2)
G.squaredNorm()//G(0)^2 + G(1)^2 + G(2)^2

// solve Ax = b
// A += J.transpose() * J
// b = J.transpose() * J
delta_bg = A.ldlt().solve(b);

for (auto it = feature.begin(), it_next = feature.begin();
	it != feature.end(); it = it_next)
{
	it_next++;
	feature.erase(it);
}

//LL分解
Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>> (pre_integration->covariance.inverse()).matrixLLT().transpose();

//矩阵中最大的数
R.maxCoeff()              // max(R(:))


// templace例子
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }
    
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        

    cv::FileStorage fs(APRIL_VIO_OUTPUT, cv::FileStorage::WRITE);
    int  i = 2;
    fs << "index" << frames[WINDOW_SIZE-1]->id; 
    fs.release();

// 迭代删除元素
while ( iter0 != image0Buf.end() && iter0->t < iter1->t ){
    iter0 =  image0Buf.erase( iter0 ) ;
    
// 快速reset
    std::fill(grid_occupancy_.begin(), grid_occupancy_.end(), false);
}
*/