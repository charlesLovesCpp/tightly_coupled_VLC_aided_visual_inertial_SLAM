#include "tag_detector.h"

int TagDetector::n_id = 0;

int channels[] = {0,1};
cv::Mat dstHist; 
int histSize = 200;      
float histR[] = {0,255}; 
const float *histRange = histR;

// cv::Point2f& pt_tl = prev_tags[0].pts[0];
// cv::Point2f& pt_br = prev_tags[0].pts[2];
//     cv::Rect rect = cv::Rect(pt_tl, pt_br);
cv::Rect rect;
//cv::Mat at_img = img(rect);	// selected apriltag
cv::Mat targ_at_img;
int track_num = 1;		//prev_tags.size()

// cv::Mat at_img = img(rect);	// selected apriltag
// cv::cvtColor(at_img, targ_at_img, CV_BGR2HSV);	
// cv::calcHist(&targ_at_img,2,channels,cv::Mat(),dstHist,1,&histSize,&histRange,true,false);           
// cv::normalize(dstHist,dstHist,0,255,CV_MINMAX);
    
void TagDetector::init()
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 0, "Spend more time trying to align edges of tags");
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
    
}

bool flag = false;

void TagDetector::readImage(cv::Mat& _img, double _cur_time)
{
    cv::Mat img, img_gray;
    cv::cvtColor(_img, img_gray, CV_BGR2GRAY);
    
    cur_time = _cur_time;
    if (EQUALIZE) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(img_gray, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());	
    } else
	img = img_gray;
    
    if (prev_img.empty()) {
	prev_img = cur_img = img_gray;
    } else {
	cur_img = img_gray;
    }
    
    cur_tags.clear();
    
    if (prev_tags.size() > 0) {
	//TODO Tracking algorithm
	
	if (cur_tags.size() > 0)
	    ROS_INFO("Track tags successfully");
    }
    
    // Track previous features
    cur_pts.clear();
    if (prev_pts.size() > 0) {
	vector<uchar> status;
	vector<float> err;
	cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
	
	for (int i = 0; i < int(cur_pts.size()); ++i) {
	    if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
	}
	reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
	reduceVector(prev_un_pts, status);
        reduceVector(track_cnt, status);
    }

    for (auto &n : track_cnt)
	n++;
    //ROS_INFO("%d features are tracked", cur_pts.size());
    
    if (PUB_THIS_FRAME && !flag) {
	flag = true;
	
	cv::Mat mask = img;
// 	if (cur_tags.size() > 0) {
// 	    setTagMask(img_gray);
// 	}
	   
	//TicToc t_detect; 
	detectNewTags(img_gray);
	//ROS_INFO("Detecting tags cost: %f", t_detect.toc());
// 	if (cur_tags.size() > 0 && !flag) {
// 	    rect = cv::Rect(cur_tags[0].pts[0], cur_tags[0].pts[2]);
// 	    
// 	    cv::Mat at_img = _img(rect);	// selected apriltag
// 	    cv::cvtColor(at_img, targ_at_img, CV_BGR2HSV);	
// 	    cv::calcHist(&targ_at_img,2,channels,cv::Mat(),dstHist,1,&histSize,&histRange,true,false);           
// 	    cv::normalize(dstHist,dstHist,0,255,CV_MINMAX);
// 	    
// 	    flag = true;
// 	}
	
	// Reject outliers by corners of tags
// 	if (cur_tags.size() > 0)
// 	    rejectWithTags();
// 	else
	int n = cur_pts.size();
	
	rejectWithF();
	
// 	ROS_INFO("after rejectWithF, %d -> %d", n, cur_pts.size());
	
// 	// Pick out 4 corners of each cur_tag and cur_pts tracked from prev_pts
	setFeatureMask(mask);
	
	int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
// 	ROS_INFO("n_max_cnt: %d", n_max_cnt);
	if (n_max_cnt > 0) {
	    if (mask.type() != CV_8UC1)
		ROS_WARN("Wrong mask type");
	    if (mask.size() != cur_img.size())
		ROS_WARN("Wrong image size");
	    cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
	} else
	    n_pts.clear();
	
	addPoints();
	
    }
    
    
    undistortedPoints();
    
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_tags = cur_tags;
    
    // Draw on the input image
    draw(_img, cur_tags); 
}

void TagDetector::rejectWithF()
{
    if (cur_pts.size() >= 8) {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(prev_pts.size()), un_forw_pts(cur_pts.size());
        for (unsigned int i = 0; i < prev_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
	    // liftProjective makes calibration for feature points. After calibration, we can know the true point positions represented in camera frame.
	    // Then we project them to our true image frame where camera parameters are integer and known.
            m_camera->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = prev_pts.size();
        reduceVector(prev_pts, status);
	reduceVector(cur_pts, status);
        reduceVector(prev_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());	
    }
}

void TagDetector::rejectWithTags()
{

    
}

void TagDetector::setFeatureMask(cv::Mat& mask)
{
    mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    
    for (auto& it : cur_pts) {
	if (mask.at<uchar>(it) == 255)
	    cv::circle(mask, it, MIN_DIST, 0, -1);
    }
    
//     for (auto& tag : cur_tags) {
// 	cv::Point cpt((tag.pts[2].x + tag.pts[0].x)/2, (tag.pts[2].y + tag.pts[0].y)/2);
// 	if (mask.at<uchar>(cpt) == 255)
// 	    cv::circle(mask, cpt, MIN_DIST/2, 0, -1);
//     }
    
    
    
/*    
    // following code means to sort features with order of track number.
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    for (int i = 0; i < int(cur_pts); ++i) {
	cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));
    }
    
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>>& a, const pair<int, pair<cv::Point2f, int>>& b) 
    {
	return a.first > b.first;// a > b, a is placed in front of b.
    });
    
    cur_pts.clear();
    ids.clear();
    track_cnt.clear();
    
    for (auto& it : cnt_pts_id) {
	if (mask.at<uchar>(it.second.first) == 255) {
	    cur_pts.push_back(it.second.first);
	    ids.push_back(it.second.second);
	    track_cnt.push_back(it.first);
	    cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
	}
    } */
}

void TagDetector::undistortedPoints()
{
    prev_un_pts.clear();
    cur_un_pts.clear();
    pts_velocity.clear();
    //prev_un_pts_map.clear();  
    
    double dt = cur_time - prev_time;
    
    // Compute undistorted 3d points and velocity of features
    for (int i = 0; i < int(cur_pts.size()); ++i) {
	Vector2d xy;
	Vector3d xyz;
	
	xy = Vector2d(cur_pts[i].x, cur_pts[i].y);
	m_camera->liftProjective(xy, xyz);
	cur_un_pts.push_back(cv::Point2f(xyz.x() / xyz.z(), xyz.y() / xyz.z()));
	pts_velocity.push_back(cv::Point2f(0, 0));
	//cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(xyz.x() / xyz.z(), xyz.y() / xyz.z())));
	
// 	if (ids[i] != -1) {
// 	    // ids[i] != -1 means this is not a new point.
// 	    xy = Vector2d(prev_pts[i].x, prev_pts[i].y);
// 	    m_camera->liftProjective(xy, xyz);
// 	    prev_un_pts.push_back(cv::Point2f(xyz.x() / xyz.z(), xyz.y() / xyz.z()));
// 	    //prev_un_pts_map.insert(make_pair(ids[i], cv::Point2f(xyz.x() / xyz.z(), xyz.y() / xyz.z())));	    
// 	    
// 	    // Compute velocity
// 	    double v_x = (cur_un_pts[i].x - prev_un_pts[i].x) / dt;
// 	    double v_y = (cur_un_pts[i].y - prev_un_pts[i].y) / dt;
// 	    pts_velocity.push_back(cv::Point2f(v_x, v_y));
// 	} else {
// 	    pts_velocity.push_back(cv::Point2f(0, 0));
// 	}
    }
    
    // Compute undistorted 3d corners and velocity of tags
    
    for (int i = 0; i < int(cur_tags.size()); ++i) {	
	for (int j = 0; j < 4; ++j) {
	    Vector2d xy;
	    Vector3d xyz;
	    xy = Vector2d(cur_tags[i].pts[j].x, cur_tags[i].pts[j].y);
	    m_camera->liftProjective(xy, xyz);
	    cur_tags[i].un_pts.push_back(cv::Point2f(xyz.x() / xyz.z(), xyz.y() / xyz.z()));	    
	}
    }
}

bool TagDetector::detectNewTags(cv::Mat& mask)
{  
    // Make an image_u8_t header for the Mat data
    image_u8_t im = { .width = mask.cols,
	.height = mask.rows,
	.stride = mask.cols,
	.buf = mask.data
    };
    
    zarray_t *detections = apriltag_detector_detect(tag_detector, &im);

    if (zarray_size(detections) == 0)
	return false;

    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++) {
	apriltag_detection_t *det;
	zarray_get(detections, i, &det);
   	
	vector<cv::Point2f> pts;
	pts.push_back(cv::Point2f(det->p[0][0], det->p[0][1]));	// topleft
	pts.push_back(cv::Point2f(det->p[1][0], det->p[1][1]));	// topright
	pts.push_back(cv::Point2f(det->p[2][0], det->p[2][1]));	// bottomright
	pts.push_back(cv::Point2f(det->p[3][0], det->p[3][1]));	// bottomleft
	cur_tags.push_back(Tag(det->id, pts));
	
	//ROS_INFO("Detect a new tag!");
	//cur_tags.insert(make_pair(det->id, Tag(det->id, pts));

    }
    zarray_destroy(detections);
    return true;    
}


 
void TagDetector::setTagMask(cv::Mat& mask)
{
    for (auto& tag : cur_tags) {

	Vector2d d_xy(abs(tag.pts[2].x - tag.pts[0].x), abs(tag.pts[2].y - tag.pts[0].y));
	cv::Point cpt((tag.pts[2].x + tag.pts[0].x)/2, (tag.pts[2].y + tag.pts[0].y)/2);
	int radius = cvRound(d_xy.norm())/2;
	cv::circle(mask, cpt, radius, 0, -1);
	ROS_INFO("setTagMask");
    }
    
}

void TagDetector::draw(cv::Mat& img, const vector<Tag>& tags)
{
    for (auto& tag : tags) {
	
	cv::line(img, tag.pts[0], tag.pts[1], cv::Scalar(0, 0, 0xff), 2);
	cv::line(img, tag.pts[0], tag.pts[3], cv::Scalar(0, 0, 0xff), 4);
	cv::line(img, tag.pts[1], tag.pts[2], cv::Scalar(0, 0, 0xff), 6);
	cv::line(img, tag.pts[3], tag.pts[2], cv::Scalar(0, 0, 0xff), 8);

	stringstream ss;
	ss << tag.id;
	string text = ss.str();
	
	int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontscale = 1.0;
	int baseline;
	cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
					&baseline);
	cv::putText(img, text, cv::Point((tag.pts[2].x + tag.pts[0].x)/2-textsize.width/2,
					 (tag.pts[2].y + tag.pts[0].y)/2+textsize.height/2),
		fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
	
	for (unsigned int i = 0; i < 4; ++i) {
	    stringstream corner_idx;
	    corner_idx << i;
	    string corner_idx_text = corner_idx.str();
	    cv::putText(img, corner_idx_text, cv::Point(tag.pts[i].x, tag.pts[i].y),
		    fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);	    
	}
	
	

    }
    
    
    for (unsigned int j = 0; j < cur_pts.size(); j++){
	double len = std::min(1.0, 1.0 * track_cnt[j] / WINDOW_SIZE);
	cv::circle(img, cur_pts[j], 2, cv::Scalar(0xff, 0xff, 0xff), 2);
    }
}

void TagDetector::addPoints()
{
    for (auto &p : n_pts)
    {
        cur_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void TagDetector::updateID()
{  
    for (unsigned int i = 0; i < ids.size(); ++i) {
	if (ids[i] == -1)
	    ids[i] = n_id++;	
    }
}


bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void TagDetector::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}


void TagDetector::trackPrevTagsCamShift(cv::Mat& img) 
{
    ROS_INFO("trackPrevTagsCamShift in...");
    cv::Mat imageHSV;    
    cv::Mat calcBackImage;    
    cv::cvtColor(img,imageHSV,CV_BGR2HSV);    
    cv::calcBackProject(&imageHSV,2,channels,dstHist,calcBackImage,&histRange); 
    
    cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 1000, 0.001); 
    ROS_INFO("CamShift...");
    cv::CamShift(calcBackImage, rect, criteria);   
   
    cv::Mat imageROI = imageHSV(rect); 
    targ_at_img = imageHSV(rect);   
    
    cv::calcHist(&imageROI, 2, channels, cv::Mat(), dstHist, 1, &histSize, &histRange);      
    cv::normalize(dstHist, dstHist, 0.0, 1.0, cv::NORM_MINMAX);      
    cv::rectangle(img, rect, cv::Scalar(255, 0, 0),3);    
    ROS_INFO("trackPrevTagsCamShift out...");
}

void TagDetector::trackPrevTags()
{
    vector<cv::Point2f> prev_pts;
    vector<cv::Point2f> cur_pts;
    for (auto& prev_tag : prev_tags) {
	prev_pts.insert(prev_pts.begin(), prev_tag.pts.begin(), prev_tag.pts.end());
    }
    //ROS_INFO("After adding pts, prev_pts.size(): %d", prev_pts.size());
    vector<uchar> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);

    for (int i = 0; i < int(cur_pts.size()); i++) {
	if (status[i] && !inBorder(cur_pts[i]))
	    status[i] = 0;	
    }
	
    //TODO Check tracking state
    auto it_st = status.begin();
    auto it_cp = cur_pts.begin();
    for (int i = 0; it_st != status.end(); ++i, it_st += 4, it_cp += 4) {
	uchar state;
	state = 0x0F & ( (*it_st)<<3 | *(it_st+1)<<2 | *(it_st+2)<<1 | *(it_st+3)<<0 );
	cv::Point2f& pt_tl = *(it_cp);
	cv::Point2f& pt_tr = *(it_cp+1);
	cv::Point2f& pt_br = *(it_cp+2);
	cv::Point2f& pt_bl = *(it_cp+3);
	
	vector<cv::Point2f> pts;
	//ROS_INFO("state: %d", state);
	switch (state) {
	    case 0x0F:
		pts.assign(it_cp, it_cp+4);
		cur_tags.push_back(Tag(prev_tags[i].id, pts));
		//ROS_INFO("track ID: %d", prev_tags[i].id);
		break;
	    case 0x07://7
		pt_tl = cv::Point2f( pt_tr.x + (pt_bl.x - pt_br.x), pt_tr.y + (pt_bl.y - pt_br.y));
		pts.assign(it_cp, it_cp+4);
		cur_tags.push_back(Tag(prev_tags[i].id, pts));
		//ROS_INFO("track ID: %d", prev_tags[i].id);
		break;
	    case 0x0B://11
		pt_tr = cv::Point2f( pt_tl.x + (pt_br.x - pt_bl.x), pt_tl.y + (pt_br.y - pt_bl.y));
		pts.assign(it_cp, it_cp+4);
		cur_tags.push_back(Tag(prev_tags[i].id, pts));
		//ROS_INFO("track ID: %d", prev_tags[i].id);
		break;
	    case 0x0E://14
		pt_br = cv::Point2f( pt_bl.x + (pt_tl.x - pt_tr.x), pt_bl.y + (pt_tl.y - pt_tr.y));
		pts.assign(it_cp, it_cp+4);
		cur_tags.push_back(Tag(prev_tags[i].id, pts));	
		//ROS_INFO("track ID: %d", prev_tags[i].id);
		break;
	    case 0x0D://13
		pt_bl = cv::Point2f( pt_br.x + (pt_tr.x - pt_tl.x), pt_br.y + (pt_tr.y - pt_tl.y));
		pts.assign(it_cp, it_cp+4);
		cur_tags.push_back(Tag(prev_tags[i].id, pts));
		//ROS_INFO("track ID: %d", prev_tags[i].id);
		break;
	    default:
		ROS_DEBUG("No enough tag corners");
		break;
	}
    }  
}