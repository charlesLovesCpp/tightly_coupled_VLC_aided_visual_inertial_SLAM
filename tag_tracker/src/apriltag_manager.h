#pragma once

#include <ros/ros.h>

#include <iostream>
#include <string>
#include <list>
#include <unordered_map>
#include <assert.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include <apriltag.h>
#include <common/getopt.h>
#include <tag36h11.h>
#include <tag36h10.h>
#include <tag36artoolkit.h>
#include <tag25h9.h>
#include <tag25h7.h>

#include "parameters.h"

using namespace std;
using namespace Eigen;

namespace aprilTag {

class AprilTagPerFrame {
    
public:
    AprilTagPerFrame(int _frame_id, Eigen::Vector2d _tag_size_img,
		     const vector<Eigen::Vector2d> &_tag_corner_uvs, 
		     const vector<Eigen::Vector3d> &_tag_corner_points
		    ): frame_id(_frame_id), tag_size_img(_tag_size_img){
			assert(_tag_corner_uvs.size() == 4 && _tag_corner_points.size() == 4);
			
			tag_corner_uvs.assign(_tag_corner_uvs.begin(), _tag_corner_uvs.end()); 
			tag_corner_points.assign(_tag_corner_points.begin(), _tag_corner_points.end()); 
		    };
    
    vector<Eigen::Vector2d> tag_corner_uvs;
    vector<Eigen::Vector3d> tag_corner_points;
    Eigen::Vector2d velocity;
    int frame_id;
    Eigen::Vector2d tag_size_img;
    double scale;
};    
    
    
class AprilTagPerId {
    
public:
    
    AprilTagPerId(int _tag_id, string _tag_text, Eigen::Vector2d _tag_size_w,
		  Eigen::Vector3d _pos_w_t, Eigen::Quaterniond _rot_w_t): tag_id(_tag_id), tag_text(_tag_text),
		  tag_size_w(_tag_size_w), pos_w_t(_pos_w_t), rot_w_t(_rot_w_t) {
		      tag_corners_w_t.resize(4);
		      tag_corners_w_t[0] = pos_w_t;								// topleft
		      tag_corners_w_t[1] = rot_w_t * Vector3d(tag_size_w.x(), 0, 0) + pos_w_t;			// topright
		      tag_corners_w_t[2] = rot_w_t * Vector3d(0, tag_size_w.y(), 0) + pos_w_t;			// bottomleft
		      tag_corners_w_t[3] = rot_w_t * Vector3d(tag_size_w.x(), tag_size_w.y(), 0) + pos_w_t;	//bottomright
		  
		}
    
    vector<Eigen::Vector3d> tag_corners_w_t;
    Eigen::Vector3d pos_w_t;
    Eigen::Quaterniond rot_w_t;
    Eigen::Vector2d tag_size_w;
    int tag_id;
    string tag_text;
    vector<AprilTagPerFrame> tag_per_frame;
};


class AprilTagManager {
public:
    //AprilTagManager();
    //~AprilTagManager();
    
    // detect aprilTags and put them into tags.
    bool detectTags(cv::Mat &image);				
    
    // initialize options
    void initOpt();		
    
    //void writeDictionary(const AprilTagPerId &april_tag);
    
    void clear();

    
    apriltag_detector_t *tag_detector;
    apriltag_family_t *tag_family;
    
    cv::Mat image;						 
    list<AprilTagPerId> tags;
    std::unordered_map<int, AprilTagPerId> tag_dictionary;
}; 


}















