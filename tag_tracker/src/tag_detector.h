#pragma once

#include <ros/ros.h>
#include <iostream>
#include <string>

#include <queue>
#include <execinfo.h>
#include <csignal>

#include <apriltag.h>
#include <common/getopt.h>
#include <tag36h11.h>
#include <tag36h10.h>
#include <tag36artoolkit.h>
#include <tag25h9.h>
#include <tag25h7.h>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class Tag 
{
public:  
    int id;
    vector<cv::Point2f> pts;
    vector<cv::Point2f> un_pts;
    cv::Point2f velocity;
    
    Tag(int _id, vector<cv::Point2f> _pts, cv::Point2f velocity = cv::Point2f(0, 0)) :
	id(_id), pts(_pts), velocity(velocity)
    {}
    
    Tag(int _id, vector<cv::Point2f> _pts, vector<cv::Point2f> _un_pts, cv::Point2f velocity = cv::Point2f(0, 0)) :
	id(_id), pts(_pts), un_pts(_un_pts), velocity(velocity)
    {}
};

class TagDetector
{
public:
    cv::Mat			cur_img; 
    vector<Tag>			cur_tags;
    //map<int, Tag>		cur_tags;
    vector<cv::Point2f> 	cur_pts;
    vector<cv::Point2f>		cur_un_pts;
    
    cv::Mat			prev_img;
    vector<Tag>			prev_tags; 
    //map<int, Tag>		prev_tags;
    vector<cv::Point2f> 	prev_pts;
    vector<cv::Point2f>		prev_un_pts;
    
    vector<cv::Point2f>		pts_velocity;
    vector<int>			ids;
    vector<int>			track_cnt;
    
    map<int, cv::Point2f>	cur_un_pts_map;
    map<int, cv::Point2f> 	prev_un_pts_map;
    
    camodocal::CameraPtr 	m_camera;
    double 			cur_time;
    double 			prev_time;
    
    static int 			n_id;
    vector<cv::Point2f>		n_pts;
    
    apriltag_detector_t*	tag_detector;
    apriltag_family_t*		tag_family;
     
    void init();
    
    void readImage(cv::Mat& _img, double _time);
    
    void rejectWithF();
    
    void rejectWithTags();
    
    void setFeatureMask(cv::Mat& mask);

    void undistortedPoints();
    
    void addPoints();
    
    void updateID();
    
    // Detect new aprilTags (after setting mask)
    bool detectNewTags(cv::Mat& mask);
    
    // Track previous aprilTags
    void trackPrevTags();
    
    void trackPrevTagsCamShift(cv::Mat& img);
    
    void setTagMask(cv::Mat& mask);
    
    void draw(cv::Mat& img, const vector<Tag>& tags);
    
    void readIntrinsicParameter(const string &calib_file);
    
};
