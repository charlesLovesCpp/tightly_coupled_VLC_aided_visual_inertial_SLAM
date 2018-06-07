#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>

#include "parameters.h"

#include "estimator.h"
#include "feature.h"
#include "frame.h"
#include "aprilTag.h"
#include "factor/integration_base.h"

using namespace basic;

typedef vector<pair<vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> MeasureGroup;

Estimator estimator;


// thread
std::condition_variable con;
std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

// data buffer
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;

// flag
bool init_feature = false;
bool init_imu = false;
bool init_tag_flag = false;

// number
double current_time = 1;
double latest_time;
double last_imu_t;
int sum_of_wait = 0;

double estimator_td = 0;

MeasureGroup getMeasurements()
{
    MeasureGroup measurements;
    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator_td))
        {
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator_td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator_td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;    
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void process()
{
    while (true) {
	MeasureGroup measurements;
	std::unique_lock<std::mutex> lk(m_buf);
	con.wait(lk, [&] {
	    return (measurements = getMeasurements()).size() > 0;
	});
	lk.unlock();
	m_estimator.lock();
	
	for (auto& measurement : measurements) {
	    	    
	    auto img_msg = measurement.second;
	    
// 	    ROS_INFO("current window: %d", estimator.window_counter);
	    
	    // Add observations into estimator	    
	    map<int, vector<Eigen::Matrix<double, 7, 1>>> apriltags;
	    map<int, Eigen::Matrix<double, 7, 1>> features;
	    for (unsigned int i = 0; i < img_msg->points.size();) {
		// Tag processing
		if (img_msg->channels[0].values[i] == FeatureType::TAG) {
		    ROS_ASSERT(img_msg->channels[1].values[i] == img_msg->channels[1].values[i+3]);

		    int tag_id = img_msg->channels[1].values[i];
		    
		    vector<Eigen::Matrix<double, 7, 1>> tag_pts;
		    for (unsigned int j = 0; j < 4; ++j) {
			double x = img_msg->points[i+j].x;
			double y = img_msg->points[i+j].y;
			double z = img_msg->points[i+j].z;
			double p_u = img_msg->channels[2].values[i+j];
			double p_v = img_msg->channels[3].values[i+j];
			double velocity_x = img_msg->channels[4].values[i+j];
			double velocity_y = img_msg->channels[5].values[i+j];	
			ROS_ASSERT(z == 1);
			
			Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
			xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
			tag_pts.push_back(xyz_uv_velocity);
		    }
		    apriltags[tag_id] = tag_pts;
		    i += 4;
		    continue;
		}
	
		// Feature processing
		if (img_msg->channels[0].values[i] == FeatureType::POINT) {  
		    int feature_id = img_msg->channels[1].values[i];
		    double x = img_msg->points[i].x;
		    double y = img_msg->points[i].y;
		    double z = img_msg->points[i].z;
		    double p_u = img_msg->channels[2].values[i];
		    double p_v = img_msg->channels[3].values[i];
		    double velocity_x = img_msg->channels[4].values[i];
		    double velocity_y = img_msg->channels[5].values[i];
		    ROS_ASSERT(z == 1);	
		    
		    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
		    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
		    features[feature_id] = xyz_uv_velocity;
		    
		    i++;
		    // ROS_INFO("Receive point: %d", feature_id);
		}
		
		// ROS_INFO("Detected feature number: %d", features.size());
	    }

	    estimator.processImage(apriltags, features, img_msg->header);

	    
	    // Add imu measurements into estimator
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
	    for (auto &imu_msg : measurement.first) {
		double t = imu_msg->header.stamp.toSec();
		double img_t = img_msg->header.stamp.toSec() + estimator_td;
		
		if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;// time difference of imu measurements
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
		    
		    //TODO processIMU
		    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));		    
                } else {
		    //TODO Deal with td intepolation.
		}
	    }
	   
// 	    Eigen::Matrix3d delta_R = (estimator.frames[estimator.window_counter]->pre_integration->delta_q).toRotationMatrix();
// 	    Vector3d delta_t = estimator.frames[estimator.window_counter]->pre_integration->delta_p;
// 	    ROS_INFO_STREAM("delta R : " << std::endl << delta_R);   
// 	    ROS_INFO_STREAM("delta T : " << std::endl << delta_t.transpose());
	    
// 	    ROS_INFO("%d IMU measurements are pre-integrated.", estimator.frames[estimator.window_counter]->pre_integration.dt_buf.size());
	    
	    
	    // Start estimating
	    estimator.start();
	    
	}
	m_estimator.unlock();
	m_buf.lock();
	m_state.lock();
/*        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();*/	
	m_state.unlock();
        m_buf.unlock();
    }
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle pnh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(pnh);
    estimator.setParameter();
    
    ros::Subscriber sub_imu = pnh.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = pnh.subscribe("/tag_tracker/feature", 2000, feature_callback);	

    std::thread measurement_process{process};
    ros::spin();
    
    return 0;
}