#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};
Eigen::Vector3d g{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string APRIL_VIO_OUTPUT;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;
double TAG_SIZE;
double SCALE_TO_METRIC;
map<int, pair<double, pair<Vector3d, Eigen::Matrix3d>>> GLOBAL_TAGS_DIC;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    std::string april_folder;
    // return <param name="config_file" type="string" value="$(arg config_path)" />
    // where <arg name="config_path" default = "$(find feature_tracker)/../config/euroc/euroc_config.yaml" />
    config_file = readParam<std::string>(n, "config_file");
    april_folder = readParam<std::string>(n, "vins_folder");
    // open or stored data in a file with xml or yaml extensions.
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    // for a string, we use >>.
    fsSettings["imu_topic"] >> IMU_TOPIC;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    //fsSettings["april_output_path"] >> APRIL_VIO_OUTPUT;
    APRIL_VIO_OUTPUT = april_folder;
//     string file_data("true_data.yaml");
//     cv::FileStorage fs(april_folder, cv::FileStorage::WRITE);
//     int  i = 1;
//     fs << "index" << i; 
//     fs.release();
    
    
    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    // if there is no initial extrinsic parameters, we need to calibrate it at first.
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
	    // the calibrated extrinsic parameters are output in this path.
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

	// maybe this is the transformation bewteen camera frame and IMU frame.
        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);// the initial guess for extrinsic rotation.
        TIC.push_back(eigen_T);// the initial guess for extrinsic translation.
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    // initial value of time offset.
    TD = fsSettings["td"];
    // online estimate time offset between camera and imu
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);// in this implementation, it is assumed that the sensors are synchronized.

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    TAG_SIZE = 4.2;
    SCALE_TO_METRIC = 0.01;
 
    int tag_num = fsSettings["total_tag_num"];
    ROS_INFO_STREAM("--------------Total tag number: " << tag_num << "--------------");
    for (int i = 0; i < tag_num; ++i) {
	// Set tag ID
	int tag_id;
	stringstream ss_id;
	ss_id << "tag_id_" << i;
	string text_id = ss_id.str();
	tag_id = fsSettings[text_id];
	ROS_INFO_STREAM("Tag id: " << tag_id);
	
	// Set tag size
	double tag_size;
	stringstream ss_size;
	ss_size << "tag_size_" << i;
	string text_size = ss_size.str();
	tag_size = fsSettings[text_size];	
	ROS_INFO_STREAM("Tag size: " << tag_size);
	
	// Set rotation
	Eigen::Matrix3d eigen_R;
	eigen_R.setIdentity();
	
	// Set position
	stringstream ss_p;
	ss_p << "tag_position_" << i;
	string text_p = ss_p.str();
	cv::Mat cv_T;
	fsSettings[text_p] >> cv_T;
	Eigen::Vector3d eigen_T;
	cv::cv2eigen(cv_T, eigen_T);
	ROS_INFO_STREAM("Tag position: " << eigen_T.transpose());

	GLOBAL_TAGS_DIC[tag_id] = make_pair(tag_size, make_pair(eigen_T, eigen_R));
	
    }
    
    fsSettings.release();
}
