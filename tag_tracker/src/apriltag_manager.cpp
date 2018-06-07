#include "apriltag_manager.h"

namespace aprilTag {
 
void AprilTagManager::initOpt() {
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    // Initialize tag detector with options
    tag_family = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11"))
        tag_family = tag36h11_create();
    else if (!strcmp(famname, "tag36h10"))
        tag_family = tag36h10_create();
    else if (!strcmp(famname, "tag36artoolkit"))
        tag_family = tag36artoolkit_create();
    else if (!strcmp(famname, "tag25h9"))
        tag_family = tag25h9_create();
    else if (!strcmp(famname, "tag25h7"))
        tag_family = tag25h7_create();
    else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }
    tag_family->black_border = getopt_get_int(getopt, "border");

    tag_detector = apriltag_detector_create();
    apriltag_detector_add_family(tag_detector, tag_family);
    tag_detector->quad_decimate = getopt_get_double(getopt, "decimate");
    tag_detector->quad_sigma = getopt_get_double(getopt, "blur");
    tag_detector->nthreads = getopt_get_int(getopt, "threads");
    tag_detector->debug = getopt_get_bool(getopt, "debug");
    tag_detector->refine_edges = getopt_get_bool(getopt, "refine-edges");
    tag_detector->refine_decode = getopt_get_bool(getopt, "refine-decode");
    tag_detector->refine_pose = getopt_get_bool(getopt, "refine-pose");
    
    
    AprilTagPerId april_tag_0(0, "origin", Vector2d(1, 1), Vector3d(0, 0, 0), Quaterniond(1, 0, 0, 0));
    AprilTagPerId april_tag_1(1, "1A", Vector2d(1, 1), Vector3d(-50, 0, 0), Quaterniond(1, 0, 0, 0));
    AprilTagPerId april_tag_2(2, "2A", Vector2d(1, 1), Vector3d(0, 50, 0), Quaterniond(1, 0, 0, 0));
    AprilTagPerId april_tag_3(3, "3A", Vector2d(1, 1), Vector3d(50, 0, 0), Quaterniond(1, 0, 0, 0));
    AprilTagPerId april_tag_4(4, "4A", Vector2d(1, 1), Vector3d(0, 50, 0), Quaterniond(1, 0, 0, 0));
    tag_dictionary.insert(make_pair(0, april_tag_0));
    tag_dictionary.insert(make_pair(1, april_tag_1));
    tag_dictionary.insert(make_pair(2, april_tag_2));
    tag_dictionary.insert(make_pair(3, april_tag_3));
    tag_dictionary.insert(make_pair(4, april_tag_4));
}

bool AprilTagManager::detectTags(cv::Mat &image) {
    // Make an image_u8_t header for the Mat data
    image_u8_t im = { .width = image.cols,
	.height = image.rows,
	.stride = image.cols,
	.buf = image.data
    };

    zarray_t *detections = apriltag_detector_detect(tag_detector, &im);
    
    if (zarray_size(detections) == 0)
	return false;
    
    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++) {
	apriltag_detection_t *det;
	zarray_get(detections, i, &det);
	
	std::unordered_map<int, AprilTagPerId>::const_iterator it_apr = tag_dictionary.find(det->id);
	if (it_apr == tag_dictionary.end()) {
	    continue;
	}
	    	
	    	
// 	// Store 4 corner points in image
// 	vector<Vector2d> uvs(4);
// 	uvs[0] = Vector2d(det->p[0][0], det->p[0][1]);
// 	uvs[1] = Vector2d(det->p[1][0], det->p[1][1]);
// 	uvs[2] = Vector2d(det->p[3][0], det->p[3][1]);
// 	uvs[3] = Vector2d(det->p[2][0], det->p[2][1]);
// 	
// 	
// 	
// 	// Points represented in camera frame
// 	vector<Vector3d> points(4);
// 	points[0] = 
// 	
// 	// Create AprilTagPerFrame to store all information
// 	AprilTagPerFrame april_tag_per_frame(0, abs(det->p[0][0] - det->p[0][1]), uvs, points);
// 	
// 	bool flag = 0;
// 	for (auto &it : tags) {
// 	    if (it->tag_id == it_apr->first) {
// 		flag = 1;
// 		break;
// 	    }
// 	}
// 	
// 	if (flag == 1) {
// 	    //it->tag_per_frame.push_back(april_tag_per_frame);
// 	} else {
// 	    tags.push_back(it_apr->second);
// 	}
	

	
	//TODO Add AprilTagPerFrame

	
	
	

	
	
	
	string text = it_apr->second.tag_text;
	
	cv::line(image, cv::Point(det->p[0][0], det->p[0][1]),
		    cv::Point(det->p[1][0], det->p[1][1]),
		    cv::Scalar(0, 0xff, 0), 2);
	cv::line(image, cv::Point(det->p[0][0], det->p[0][1]),
		    cv::Point(det->p[3][0], det->p[3][1]),
		    cv::Scalar(0, 0, 0xff), 2);
	cv::line(image, cv::Point(det->p[1][0], det->p[1][1]),
		    cv::Point(det->p[2][0], det->p[2][1]),
		    cv::Scalar(0xff, 0, 0), 2);
	cv::line(image, cv::Point(det->p[2][0], det->p[2][1]),
		    cv::Point(det->p[3][0], det->p[3][1]),
		    cv::Scalar(0xff, 0, 0), 2);


	int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontscale = 1.0;
	int baseline;
	cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
					&baseline);
	cv::putText(image, text, cv::Point(det->c[0]-textsize.width/2,
				    det->c[1]+textsize.height/2),
		fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
    }
    zarray_destroy(detections);
    
    return true;
}

}