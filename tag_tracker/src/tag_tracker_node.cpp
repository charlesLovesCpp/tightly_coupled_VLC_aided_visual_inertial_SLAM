#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "parameters.h"

#include <iostream>

//#include "apriltag_manager.h"

#include "tag_detector.h"

TagDetector tagDetector;

ros::Publisher pub_img;

double first_image_time;
double last_image_time;
bool first_image_flag = true;
int pub_count = 1;
bool init_pub = 0;


void img_callback(const sensor_msgs::ImageConstPtr &img_msg) {
    
    if (first_image_flag) {
	first_image_flag = false;
	first_image_time = img_msg->header.stamp.toSec();
	last_image_time = img_msg->header.stamp.toSec();
	return;
    }
    
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time) {
	ROS_WARN("Discontinue image!");
	first_image_flag = true;
	last_image_time = 0;
	pub_count = 1;
	return;
    }
    
    last_image_time = img_msg->header.stamp.toSec();
    
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ) {
	PUB_THIS_FRAME = true;
	if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ) {
	    first_image_time = img_msg->header.stamp.toSec();
	    pub_count = 0;
	}
    } else
	PUB_THIS_FRAME = false;
    
    
    cv_bridge::CvImagePtr cv_ptr;
    try {
	cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
    }

    //TicToc t_r;
    
    cv::Mat img_bgr = cv_ptr->image;
    
    // get cur_tags, cur_pts
    tagDetector.readImage(img_bgr, img_msg->header.stamp.toSec());

    tagDetector.updateID();

     
    
    if (PUB_THIS_FRAME && false) {
	pub_count++;
	
	sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
	sensor_msgs::ChannelFloat32 type_of_point;
	sensor_msgs::ChannelFloat32 id_of_point;
	sensor_msgs::ChannelFloat32 u_of_point;
	sensor_msgs::ChannelFloat32 v_of_point;
	sensor_msgs::ChannelFloat32 velocity_x_of_point;
	sensor_msgs::ChannelFloat32 velocity_y_of_point;  
	
	feature_points->header = img_msg->header;
	feature_points->header.frame_id = "world";
	
	FeatureType type;
	
	auto& tags = tagDetector.cur_tags;	
	for (auto& tag : tags) {
	    for (int i = 0; i < 4; ++i) {
		type = TAG;
		type_of_point.values.push_back(type);
		
		geometry_msgs::Point32 p;
		p.x = tag.un_pts[i].x;
		p.y = tag.un_pts[i].y;
		p.z = 1;
		feature_points->points.push_back(p);
		
		id_of_point.values.push_back(tag.id);
		u_of_point.values.push_back(tag.pts[i].x);
		v_of_point.values.push_back(tag.pts[i].y);
		velocity_x_of_point.values.push_back(tag.velocity.x);
		velocity_y_of_point.values.push_back(tag.velocity.y);		
	    }
	}
	
	auto& pts = tagDetector.cur_pts;
	auto& un_pts = tagDetector.cur_un_pts;
	auto &ids = tagDetector.ids;
        auto &pts_velocity = tagDetector.pts_velocity;
	for (int i = 0; i < pts.size(); ++i) {
	    if (tagDetector.track_cnt[i] > 1)
	    {
// 		ROS_INFO("track_cnt: %d", tagDetector.track_cnt[i]);
		type = POINT;
		type_of_point.values.push_back(type);
		
		geometry_msgs::Point32 p;
		p.x = un_pts[i].x;
		p.y = un_pts[i].y;
		p.z = 1;

		feature_points->points.push_back(p);
		id_of_point.values.push_back(ids[i]);
		u_of_point.values.push_back(pts[i].x);
		v_of_point.values.push_back(pts[i].y);
		velocity_x_of_point.values.push_back(pts_velocity[i].x);
		velocity_y_of_point.values.push_back(pts_velocity[i].y);
	    }	    
	}
	
	feature_points->channels.push_back(type_of_point);
	feature_points->channels.push_back(id_of_point);
	feature_points->channels.push_back(u_of_point);
	feature_points->channels.push_back(v_of_point);
	feature_points->channels.push_back(velocity_x_of_point);
	feature_points->channels.push_back(velocity_y_of_point);   
    
	ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
	if (!init_pub)
	{
	    init_pub = 1;
	}
	else
	    pub_img.publish(feature_points);	
	
    }

    //ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
    
    cv::imshow("view", img_bgr);
    cv::waitKey(5);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tag_tracker");
    ros::NodeHandle pnh("~");
    
    readParameters(pnh);
    tagDetector.readIntrinsicParameter(CAM_NAMES[0]);
    tagDetector.init();
    
    ros::Subscriber sub_img;
    sub_img = pnh.subscribe(IMAGE_TOPIC, 100, img_callback);
    
    pub_img = pnh.advertise<sensor_msgs::PointCloud>("feature", 1000);
    
    ros::spin();
    return 0;
} 
