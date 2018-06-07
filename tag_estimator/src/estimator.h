#pragma once

#include "parameters.h"
#include <boost/shared_ptr.hpp>

#include "factor/pose_local_parameterization.h"
#include "factor/integration_base.h"
#include "factor/initial_factor.h"
#include "factor/imu_factor.h"
#include "factor/reprojection_factor.h"
#include "factor/const_point_factor.h"
#include "factor/point_factor.h"

#define USE_DATA_RECORDER 1 

namespace basic 
{;
class Frame;
class Feature;
class Point;
class AprilTagInstance;
class AprilTagFeature;

typedef boost::shared_ptr<Frame> FramePtr;
typedef map<int, vector<Eigen::Matrix<double, 7, 1>>> AprilTagMap;
typedef map<int, Eigen::Matrix<double, 7, 1>> FeatureMap;

class Estimator
{
public:
    
    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag marginalization_flag;
    
    FramePtr frames[(WINDOW_SIZE + 1)];
    FramePtr last_frame;
    int window_counter;
    Matrix3d ric;	// r_body_cam
    Vector3d tic;	// t_body_cam    
    Vector3d acc_0;
    Vector3d gyr_0;
    double para_Ex_Pose[SIZE_POSE];
    double s_imu_tag;			// s * t_tag_i = t_imu_i
    bool first_imu;
    bool first_tag_img;
    map<int, Point*> pts_map; 
    map<int, AprilTagInstance*> tags_map; 
    map<int, pair<double, pair<Vector3d, Eigen::Matrix3d>>> global_tags_dic;
    double initial_timestamp;
    
    ofstream file_true;
    ofstream file_pnp;
    ofstream file_go;  
    ofstream file_tag_num;
    
    Estimator();
    
    ~Estimator();
    
    void start();
    
    void setParameter();
    
    void clearState();
    
    void processIMU(const double& _dt, const Vector3d& _linear_acceleration, const Vector3d& _angular_velocity);
    
    void processImage(const AprilTagMap& _apriltag_map, const FeatureMap& _feature_map, const std_msgs::Header& _header);
    
    void slideWindow();
    
    void slideWindowOld();
    
    void slideWindowNew();
    
    
    // TAG RECOGNIZATION
    
    void loadDictionary();
    
    bool consultDictionary(const int& _id, Eigen::Matrix3d& R, Vector3d& t, double& size);

    // INITIALIZATION PART
    
    bool initialStructure();
        
    bool solveGlobalRT(int idx, Eigen::Matrix3d& R, Vector3d& t, int flag = 0, bool use_initial = false);// flag 0 -> tag + poitns, 1 -> tag only, 2 -> points only  
    
    bool solvePnPByLeds(int idx, Matrix3d& R, Vector3d& t, bool use_initial);
    
    bool getTriangulateCandidate(const vector<int>& _tag_index, int& idx);
    
    bool construct(int left_idx, int right_idx);
    
    void triangulatePointsInTwoFrames(int idx_a, const Eigen::Matrix<double, 3, 4>& Pose_a, int idx_b, const Eigen::Matrix<double, 3, 4>& Pose_b);
    
    // IMU ALIGNMENT PART
    
    bool visualInitialAlign(); 
    
    bool solveGyroscopeBias();
    
    bool linearAlignment();
    
    void RefineGravity(Vector3d& _g, VectorXd& _x);
    
    // ESTIMATION PART
    
    bool solveLocalization();
    
    bool failureDetection();
    
    bool optimization();
    
    void solvePnPByCeres();
    
    void predict();
    
    // INTERFACE PART
    
    void showPosition(const Matrix3d& _R_w_c, const Vector3d& _t_w);
    
    // TOOL
    
    bool computeParallex(int i, int j, double& parallex);
    
    void computeDistanceByImg(const Vector3d& pt_1, const Vector3d& pt_2, double& parallex);
    
    void getCorrespondingPoints(int i, int j, vector<pair<Feature*, Feature*>>& match_points);
    
    void getCorrespondingTags(int i, int j, vector<pair<AprilTagFeature*, AprilTagFeature*>>& match_tags);
    
    bool solveRTByPnP(const vector<Vector3d>& _pts, const vector<Vector2d>& _pxs, 
		      Eigen::Matrix3d& R, Vector3d& t,
		      cv::Matx33d _cam, cv::Vec4d _dist, bool flag = false);
    
    void triangulatePoint(const Eigen::Matrix<double, 3, 4>& _P_a, const Eigen::Matrix<double, 3, 4>& _P_b,
			  const Vector2d& _pt_a, const Vector2d& _pt_b, Vector3d &pt_w);
    
    void triangulatePointMap();
    
    bool projectW2C(const Matrix3d& _R_c_w, const Vector3d& _t_c, const Vector3d& _P_w, Vector3d& P_c);
    
    bool projectW2C(const Matrix3d& _R_w_b, const Vector3d& _t_w, const Matrix3d& _r_b_c, const Vector3d& _t_b_c, const Vector3d& _P_w, Vector3d& P_c);

    bool saveData(ofstream* outfile, const int& _index, const Matrix3d& _R, const Vector3d& _t, std::string _text = "");
};   

}








