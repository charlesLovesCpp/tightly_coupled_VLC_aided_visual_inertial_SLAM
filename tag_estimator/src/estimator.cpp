#include "estimator.h"

#include "frame.h"
#include "feature.h"
#include "point.h"
#include "aprilTag.h"


namespace basic 
{
 
Estimator::Estimator():
    window_counter(0),
    first_imu(true),
    first_tag_img(true),
    solver_flag(INITIAL),
    initial_timestamp(0.0)
{
    for (unsigned int i = 0; i <= WINDOW_SIZE; ++i)
	frames[i].reset(new Frame()); 
}
 
Estimator::~Estimator()
{
    for (unsigned int i = 0; i < window_counter; ++i) {
	frames[i]->clear();
	frames[i].reset();  
    }
    for (auto it = pts_map.begin(); it != pts_map.end(); ++it) {
	it->second->type = Point::TYPE_INIT;
	delete(it->second);
    }  
    for (auto it = tags_map.begin(); it != tags_map.end(); ++it) {
	delete(it->second);
    }
    pts_map.clear();
    tags_map.clear();
}

void Estimator::clearState()
{
    for (unsigned int i = 0; i <= window_counter; ++i) {
	frames[i]->clear();
	frames[i].reset(new Frame());  
    }
    for (auto it = pts_map.begin(); it != pts_map.end(); ++it) {
	it->second->type = Point::TYPE_INIT;
	delete(it->second);
    }  
    for (auto it = tags_map.begin(); it != tags_map.end(); ++it) {
	delete(it->second);
    }
    solver_flag = INITIAL;
    first_imu = true;
    first_tag_img = true;
    window_counter = 0;

    pts_map.clear();
    tags_map.clear();
    
    Frame::frame_counter = 0;
    ROS_INFO("States have been cleared!\n");
    
#ifdef USE_DATA_RECORDER     
    file_true.close();
    file_pnp.close();
    file_go.close();
    file_tag_num.close();
#endif    
}

void Estimator::start()
{
    if (solver_flag == INITIAL) {
	
	if (window_counter == WINDOW_SIZE){
	    bool result = false;
	    if ((frames[window_counter]->timestamp - initial_timestamp) > 0.2) {
		
		result = initialStructure();
		initial_timestamp = frames[window_counter]->timestamp;
		
		if (result) {
		    ROS_INFO("_________________Initialization successes!____________________\n");	
		    
		    
		    if (frames[window_counter]->apriltag_fts.size() != 0) {
			Matrix3d R_c_w;  
			Vector3d P_c;	    
			if (!solveGlobalRT(window_counter, R_c_w, P_c, 1, false)) {
			    ROS_INFO("Fail solving position from tag");
			    return;
			} else {
			    //ROS_INFO_STREAM("******AprilTag R_w_c : " << std::endl << R_c_w.transpose() * ric.transpose());
			    ROS_INFO_STREAM("******AprilTag P_w :    " << (- R_c_w.transpose() * ric.transpose() * (ric * P_c + tic)).transpose()); 
			}
		    }		    

		    for (unsigned int i = 0; i <= window_counter; ++i) {
			Matrix3d R_w_b = frames[i]->R_w_b; 
			Vector3d t_w = frames[i]->P_w;
			//ROS_INFO_STREAM("before******Global R_w_c : " << std::endl << R_w_b);
			ROS_INFO_STREAM("before******Global P_w  " << i << "  :    "  << t_w.transpose());
		    }    
		    
// 		    for (auto it = pts_map.begin(); it != pts_map.end(); ++it) {
// 			if ((*it).second->type == Point::TYPE_TRIANGULATED)
// 			    ROS_INFO_STREAM("before****** Position:  Point " << (*it).first << std::endl << (*it).second->P_w.transpose());
// 		    }  	    
		    
		    if(!solveLocalization()) {
			ROS_WARN("solveLocalization fails...!");
			clearState();
			return;	    
		    }	
		    
		    solver_flag = NON_LINEAR;
		    
		    for (unsigned int i = 0; i <= window_counter; ++i) {
			Matrix3d R_w_b = frames[i]->R_w_b; 
			Vector3d t_w = frames[i]->P_w;
			//ROS_INFO_STREAM("before******Global R_w_c : " << std::endl << R_w_b);
			ROS_INFO_STREAM("after******Global P_w  " << i << "  :    " << t_w.transpose());
		    }   			    
// 		    for (auto it = pts_map.begin(); it != pts_map.end(); ++it) {
// 			if ((*it).second->type == Point::TYPE_TRIANGULATED)
// 			    ROS_INFO_STREAM("after****** Position:  Point " << (*it).first << std::endl << (*it).second->P_w.transpose());
// 		    }  

#ifdef USE_DATA_RECORDER     
    file_true.open(APRIL_VIO_OUTPUT+"ground_true.txt");
    file_pnp.open(APRIL_VIO_OUTPUT+"pnp_data.txt");
    file_go.open(APRIL_VIO_OUTPUT+"go_data.txt");
    file_tag_num.open(APRIL_VIO_OUTPUT+"tag_num.txt");
#endif 

		} else {
		    ROS_INFO("Initialization fails!\n");
		    for (auto it = pts_map.begin(); it != pts_map.end(); ++it) {
			it->second->type = Point::TYPE_INIT;
		    }  
		    marginalization_flag = MARGIN_OLD;
		}  		
	    }    
	    slideWindow();
	} else {
	    if (first_tag_img == true) {// no tag
		if (frames[0]->apriltag_fts.empty()) {	    
		    frames[0]->clear();
		    frames[0].reset(new Frame());
		    int n_deleted_point = 0;
		    for (auto it = pts_map.begin(), it_next = pts_map.begin(); it != pts_map.end(); it = it_next) {
			it_next++;
			if (it->second->type == Point::TYPE_DELETED) {
			    n_deleted_point++;
			    pts_map.erase(it);	
			}
		    } 	    
		    return;
		} else {
		    first_tag_img = false;
		    Matrix3d R_c_w; 
		    Vector3d t_c;
		    solveGlobalRT(0, R_c_w, t_c, 1, false);  
		}

	    }
	    window_counter++;
	}
    } else if (solver_flag == NON_LINEAR){
	// clearState();  

	predict();
	
	Vector3d t_w = frames[window_counter]->P_w;
	ROS_INFO_STREAM("predict******Global P_w:  "  << t_w.transpose());	    	
	
/*	
	for (unsigned int i = 0; i <= WINDOW_SIZE; ++i) {
	    Vector3d t_w = frames[i]->P_w;
	    ROS_INFO_STREAM("before******Global P_w  " << i << "  :    "  << t_w.transpose());	    
	}*/
	
	if(!solveLocalization()) {
	    vector<int> tag_index;
	    for (unsigned int i = 0; i <= WINDOW_SIZE; ++i) {
		if (frames[i]->apriltag_fts.size() > 0) {
		    tag_index.push_back(i);
		} else
		    continue;
	    }
	    ROS_WARN("solveLocalization fails...!, there are only %d tags ", tag_index.size());
	    clearState();
	    return;	    
	}
/*	
	for (unsigned int i = 0; i <= WINDOW_SIZE; ++i) {
	    Vector3d t_w = frames[i]->P_w;
	    ROS_INFO_STREAM("after******Global P_w  " << i << "  :    "  << t_w.transpose());	    
	}*/
	
	if(failureDetection() || frames[window_counter]->apriltag_fts.size() < 1) {
	    ROS_WARN("failure detection!");	
	    clearState();
	    return;
	}
	
	ROS_INFO_STREAM("Estimated position : " << frames[window_counter]->P_w.transpose());

	slideWindow();
	
#ifdef USE_DATA_RECORDER  
    if (saveData(&file_go, frames[WINDOW_SIZE-1]->id, frames[WINDOW_SIZE-1]->R_w_b, frames[WINDOW_SIZE-1]->P_w))
	ROS_INFO("saving data successess!");
#endif
    
	ROS_INFO("__________________________location solved___________________________________________");
    }
}

void Estimator::predict()
{
	Matrix3d R_c_w;  
	Vector3d P_c;	    

	if (!solveGlobalRT(window_counter, R_c_w, P_c, 1, false)) {
	    ROS_INFO("Fail solving position from tag");
	    return;
	} else {
	    Matrix3d R_w_b = R_c_w.transpose() * ric.transpose();
	    Vector3d P_w = - R_c_w.transpose() * (ric.transpose() * tic + P_c);
	    
	    ROS_INFO_STREAM("Multiple AprilTag P_w : " << P_w.transpose()); 
#ifdef USE_DATA_RECORDER  
    if (saveData(&file_true, frames[WINDOW_SIZE]->id, R_w_b, P_w))
	ROS_INFO("saving data successess!");
    
    if (file_tag_num.is_open()) {	
	try {
	    file_tag_num << frames[WINDOW_SIZE]->id << " " << frames[WINDOW_SIZE]->apriltag_fts.size() << "\n";
	} catch (exception& e) {
	    ROS_ERROR("saving data fails...");
	}    
    } else {
	ROS_ERROR("saving data fails...");
    }
    
#endif
	}
	
    if (frames[window_counter]->apriltag_fts.size() >= 4) {	    
#ifdef USE_LED_MODE
	if (!solvePnPByLeds(window_counter, R_c_w, P_c, false)) {
	    ROS_INFO("Fail solving position from tag");
	    return;
	} 
#else
	if (!solveGlobalRT(window_counter, R_c_w, P_c, 1, false)) {
	    ROS_INFO("Fail solving position from tag");
	    return;
	}
#endif	
	Matrix3d R_w_b = R_c_w.transpose() * ric.transpose();
// 	Vector3d P_w = - R_c_w.transpose() * ric.transpose() * (ric * P_c + tic);
	Vector3d P_w = - R_c_w.transpose() * (ric.transpose() * tic + P_c);
	frames[window_counter]->setPosition(P_w);
	frames[window_counter]->setRotation(R_w_b);
	ROS_INFO_STREAM("AprilTag P_w : " << P_w.transpose()); 
	
#ifdef USE_DATA_RECORDER  
    if (saveData(&file_pnp, frames[WINDOW_SIZE]->id, R_w_b, P_w))
	ROS_INFO("saving data successess!");
#endif
	
    } else {
	double dt = frames[window_counter]->pre_integration->sum_dt;
	Vector3d delta_p = frames[window_counter]->pre_integration->delta_p;
	Vector3d delta_v = frames[window_counter]->pre_integration->delta_v;
	Matrix3d delta_r = frames[window_counter]->pre_integration->delta_q.toRotationMatrix();
	
	//TODO predict P, V by pre-integration
	Vector3d Pi = frames[window_counter-1]->P_w;
	Vector3d Vi = frames[window_counter-1]->V_w;
	Matrix3d Ri = frames[window_counter-1]->R_w_b;

	Vector3d Pj_hat = Pi + Vi * dt - 0.5 / SCALE_TO_METRIC * g * dt * dt + 1.0 / SCALE_TO_METRIC * Ri * delta_p;
	Vector3d Vj_hat = Vi - 1.0 / SCALE_TO_METRIC * g * dt + 1.0 / SCALE_TO_METRIC * Ri * delta_v;
	Matrix3d Rj_hat = Ri * delta_r;
	
	frames[window_counter]->setPosition(Pj_hat);
	frames[window_counter]->setRotation(Rj_hat);
	frames[window_counter]->setVelocity(Vj_hat);
    }
}


bool Estimator::initialStructure()
{
    vector<int> tag_index;
    for (unsigned int i = 0; i <= WINDOW_SIZE; ++i) {
	if (frames[i]->apriltag_fts.size() > 0) {
	    tag_index.push_back(i);
	} else
	    continue;
    }
    
    if (tag_index.size() < 2) {
	ROS_INFO("Less than two tag frame in the window");
	return false;
    }
    if (tag_index.size() < 1) {
	ROS_INFO("non tag frame in the window");
	clearState();
	return false;
    }
	
    
    int idx;
    if(!getTriangulateCandidate(tag_index, idx))
	return false;
    else
	ROS_INFO("Get two Tag frame:   %d   and    %d   ", idx, tag_index.back());
    
    // construct: setRotation setPosition setPoints
    if(!construct(idx, tag_index.back())) {
	ROS_INFO("Global SFM fails!");
	marginalization_flag = MARGIN_OLD;
	return false;
    }
    
    // visualInitialAlign: setBiasGyro setBaisAccelerator setVelocity setGravity setScale
    if (!visualInitialAlign()) {
	ROS_INFO("Misalign visual structure with IMU");
	return false;
    }
    
    return true;
    
}

bool Estimator::solveLocalization()
{
    if (window_counter < WINDOW_SIZE)
	return false;
    
    triangulatePointMap();

    if (!optimization())
	return false;

    return true;
}


bool Estimator::optimization()
{ 
    TicToc t_op;
    ceres::Problem problem;
    ceres::LossFunction* loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    
    /*****block*****/
    for (unsigned int i = 0; i <= WINDOW_SIZE; ++i) {
	ceres::LocalParameterization* local_parameterization = new PoseLocalParameterization();
	problem.AddParameterBlock(frames[i]->para_Pose, SIZE_POSE, local_parameterization);
	problem.AddParameterBlock(frames[i]->para_SpeedBias, SIZE_SPEEDBIAS);
    }
    
    for (unsigned int i = 0; i < NUM_OF_CAM; ++i) {
	ceres::LocalParameterization* local_parameterization = new PoseLocalParameterization();
	problem.AddParameterBlock(para_Ex_Pose, SIZE_POSE, local_parameterization);
	problem.SetParameterBlockConstant(para_Ex_Pose);
    }
    
    /*****IMU reidual*****/
    
    for (unsigned int i = 1; i <= WINDOW_SIZE; ++i) {
	if (frames[i]->pre_integration->sum_dt > 10.0)
	    continue;
	
	IMUFactor* imu_factor = new IMUFactor(frames[i]->pre_integration.get());
	problem.AddResidualBlock(imu_factor, NULL, frames[i-1]->para_Pose, frames[i-1]->para_SpeedBias,
						   frames[i]->para_Pose, frames[i]->para_SpeedBias);
    }
    

#ifdef USE_LED_MODE
// tag center blocks	
for (auto& tag_m : tags_map) {
    AprilTagInstance* tag_instance = tag_m.second;
    problem.AddParameterBlock(tag_instance->position_center, 3);	
    problem.SetParameterBlockConstant(tag_instance->position_center);
} 
// tag center residuals
for (auto& tag_m : tags_map) {
    AprilTagInstance* tag_instance = tag_m.second;
    for (auto& tag_ftr : tag_instance->tag_obs) {
	Vector3d center = tag_ftr->ftr_center;
	ConstPointFactor* point_factor = new ConstPointFactor(center);
	problem.AddResidualBlock(point_factor, loss_function, tag_ftr->frame->para_Pose, para_Ex_Pose, tag_instance->position_center);
    }
}

#else
// tag corner blocks
for (auto& tag_m : tags_map) {
    AprilTagInstance* tag_instance = tag_m.second;
    for (unsigned int i = 0; i < 4; ++i) {//charles
	problem.AddParameterBlock(tag_instance->position[i], 3);	
	problem.SetParameterBlockConstant(tag_instance->position[i]);
    }
} 
// tag corner residuals
for (auto& tag_m : tags_map) {
    AprilTagInstance* tag_instance = tag_m.second;
    for (auto& tag_ftr : tag_instance->tag_obs) {
	for (unsigned int i = 0; i < 4; ++i) {
	    Vector3d corner = tag_ftr->tag_fts[i]->pt_c;
	    ConstPointFactor* point_factor = new ConstPointFactor(corner);
	    problem.AddResidualBlock(point_factor, loss_function, tag_ftr->frame->para_Pose, para_Ex_Pose, tag_instance->position[i]);
	}
    }
}
#endif
	
/*
    for (auto& pt_m : pts_map) {
	Point* pt = pt_m.second;
	if (pt->obs.size() < 3 || pt->type != Point::TYPE_TRIANGULATED)
	    continue;

	problem.AddParameterBlock(pt->position, 3);	
// 	problem.SetParameterBlockConstant(pt->position);	
    } 

 
    for (auto& pt_m : pts_map) {
	Point* pt = pt_m.second;
	if (pt->obs.size() < 3 || pt->type != Point::TYPE_TRIANGULATED)
	    continue;

	for (auto& ftr : pt->obs) {
	    PointFactor* point_factor = new PointFactor(ftr->pt_c);
	    problem.AddResidualBlock(point_factor, loss_function, ftr->frame->para_Pose, para_Ex_Pose, pt->position);
	}
    }    */
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
//     options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.max_num_iterations = NUM_ITERATIONS;    // This will make system diverges
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);   
    
    if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03)
    {
	ROS_INFO("Optimization() : Ceres converge!");
    }
    else
    {
	ROS_INFO("Optimization() : Ceres does not converge...");
	return false;
    }   
    // Frames
    for (unsigned int i; i <= WINDOW_SIZE; ++i) {
	frames[i]->resetPRVBaBgByPara();
    }
    
    // Points
    for (auto& pt_m : pts_map) {
	Point* pt = pt_m.second;
	if (pt->type == Point::TYPE_TRIANGULATED) {
	    pt->setPositionByPara();
	}
    }    
/*    
    Vector3d ex_t(para_Ex_Pose[0], para_Ex_Pose[1], para_Ex_Pose[2]);
    ROS_INFO_STREAM("Calibrated ex_t " << ex_t.transpose());*/
    
    ROS_INFO("Optimization ceres cost : %f", t_op.toc());
    return true;
}

void Estimator::triangulatePointMap()
{
    for (auto& tag : tags_map) {

	if (tag.second->tag_obs.size() < 2)
	    continue;
	
	for (unsigned int i = 0; i < 4; ++i) {	    
	    Eigen::MatrixXd svd_A(2 * tag.second->tag_obs.size(), 4);
	    int svd_idx = 0;	
	
	    for (auto ftr_tag : tag.second->tag_obs) {
		Feature* ftr = ftr_tag->tag_fts[i];
		int frame_idx = ftr->frame->window_counter;
		
		Eigen::Matrix<double, 3, 4> P;
		Eigen::Matrix3d R_w_b = frames[frame_idx]->R_w_b;
		Eigen::Vector3d t_w_b = frames[frame_idx]->P_w;
		Eigen::Vector3d t_b_w = -R_w_b.transpose() * t_w_b;
		Eigen::Vector3d t_c_b = -ric.transpose() * tic;

		Eigen::Matrix3d R_c_w = ric.transpose() * R_w_b.transpose();
		Eigen::Vector3d t_c_w = ric.transpose() * t_b_w + t_c_b;

		P.leftCols<3>() = R_c_w;
		P.rightCols<1>() = t_c_w;	    

		//Eigen::Vector3d f = ftr->pt_c.normalized(); 
		Eigen::Vector3d f = ftr->pt_c;
		svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);	
		svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);		
	    }
	    
	    ROS_ASSERT(svd_idx == svd_A.rows());
	    Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
	    Vector3d P_w(svd_V[0]/svd_V[3], svd_V[1]/svd_V[3], svd_V[2]/svd_V[3]);
	    double dep = P_w.z();
	    
// 	    ROS_INFO_STREAM("Tag ID: " << tag.first << " Corner Index: " << i << " Position: " << P_w.transpose());
	}
    }
    
    
    
    for (auto& pt_m : pts_map) {
	Point* pt = pt_m.second;
	if (pt->obs.size() < 3 || pt->obs.front()->frame->window_counter > WINDOW_SIZE - 3)
	    continue;
	
	if (pt->type == Point::TYPE_TRIANGULATED)
	    continue;
	
	Eigen::MatrixXd svd_A(2 * pt->obs.size(), 4);
	int svd_idx = 0;	
	
	for (auto& ftr : pt->obs) {
	    int frame_idx = ftr->frame->window_counter;
	    
	    Eigen::Matrix<double, 3, 4> P;
	    Eigen::Matrix3d R_w_b = frames[frame_idx]->R_w_b;
	    Eigen::Vector3d t_w_b = frames[frame_idx]->P_w;
	    Eigen::Vector3d t_b_w = -R_w_b.transpose() * t_w_b;
	    Eigen::Vector3d t_c_b = -ric.transpose() * tic;

	    Eigen::Matrix3d R_c_w = ric.transpose() * R_w_b.transpose();
	    Eigen::Vector3d t_c_w = ric.transpose() * t_b_w + t_c_b;

	    P.leftCols<3>() = R_c_w;
	    P.rightCols<1>() = t_c_w;	    

            // Eigen::Vector3d f = ftr->pt_c.normalized(); 
	    Eigen::Vector3d f = ftr->pt_c;
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);	
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);
	}
	
	ROS_ASSERT(svd_idx == svd_A.rows());
	Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
	Vector3d P_w(svd_V[0]/svd_V[3], svd_V[1]/svd_V[3], svd_V[2]/svd_V[3]);
	double dep = P_w.z();
/*	
	bool throwout_flag = false;//charles
	for (auto& ftr : pt->obs) {
	    int frame_n = ftr->frame->window_counter;
	    Eigen::Matrix3d R_w_b = frames[frame_n]->R_w_b;
	    Vector3d P_w = frames[frame_n]->P_w;
	    Vector3d P_c;
	    if (!projectW2C(R_w_b, P_w, ric, tic, P_w, P_c)) {
		throwout_flag = true;
		continue;
	    }
	}
	
	
	if (!throwout_flag)
	    ROS_INFO("Yahoo!!!");*/

// 	pt->setPosition(P_w);
// 	ROS_INFO_STREAM("******New Point:  " << pt->id << "   Position:    " << P_w.transpose());
/*
	if (P_w.z() < 200 && P_w.z() > 1.0) {
	    pt->setPosition(P_w);
	    ROS_INFO_STREAM("******New Point:  " << pt->id << "   Position:    " << P_w.transpose());		    
	} else {
	    pt->type = Point::TYPE_INIT;
	}*/
    }
}


bool Estimator::failureDetection()
{
    if (frames[WINDOW_SIZE]->Ba.norm() > 5) {
        ROS_INFO(" big IMU acc bias estimation %f", frames[WINDOW_SIZE]->Ba.norm());
        return true;	
    }
    if (frames[WINDOW_SIZE]->Bg.norm() > 3.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", frames[WINDOW_SIZE]->Bg.norm());
        return true;
    }
    return false;
}

void Estimator::processImage(const AprilTagMap& _apriltag_map, const FeatureMap& _ftr_map, const std_msgs::Header& _header)
{ 
    //frames[window_counter].reset(new Frame(Frame::frame_counter++, window_counter, _header.stamp.toSec()));
// ROS_INFO("window_counter : %d", window_counter);    
    FramePtr frame = frames[window_counter];	
    frame->setFrameId(Frame::frame_counter++);
    frame->setWindowCounter(window_counter);
    frame->setTimestamp(_header.stamp.toSec());
    
    for (auto& raw_ftr : _ftr_map) {
	// Create new feature observation
	Feature* ftr = new Feature(frame.get(), raw_ftr.second, _header.stamp.toSec(), FeatureType::POINT);
	
	int id = raw_ftr.first;
	auto it = pts_map.find(id);
	if (it == pts_map.end()) {
	    
	    // Create new point
	    Point* pt = new Point(id);
	    
	    // Insert new Point into point map
	    pts_map[id] = pt;
	    
	    // Add observation into Point and Frame
	    ftr->setPoint(pt);
	    pt->addFrameRef(ftr);
	    frame->addFeature(ftr);
	} else {
	    // Add new observation into Point and Frame
	    ftr->setPoint(it->second);
	    it->second->addFrameRef(ftr);
	    frame->addFeature(ftr);
	}
    }
    
    int valid_tag_num = 0;
    for (auto& raw_tag : _apriltag_map) {
	// Create apriltag, generate corresponding point, and add it into frames[window_counter]
	int tag_id = raw_tag.first;

	vector<Feature*> fts;
	for (auto& tag_pt : raw_tag.second) {
	    Feature* ftr = new Feature(frame.get(), tag_pt, _header.stamp.toSec(), FeatureType::TAG);
	    fts.push_back(ftr);
	}
	
	// Create new AprilTag observation
	ROS_ASSERT(fts.size() == 4);
 	AprilTagFeature* tag_ftr = new AprilTagFeature(frame.get(), fts, _header.stamp.toSec());
	
	auto it = tags_map.find(tag_id);
	if (it == tags_map.end()) {
	    
	    double size_w; Matrix3d R; Vector3d t;    
	    if (!consultDictionary(tag_id, R, t, size_w)) {
		ROS_INFO("Detected an unregisterd AprilTag %d!", tag_id);
		continue;
	    }
	    /******************************************/
	    
	    // Create new AprilTag
	    AprilTagInstance* tag_instance = new AprilTagInstance(tag_id, size_w, R, t);

	    // Insert new AprilTag into tag map
	    tags_map[tag_id] = tag_instance;
	    
	    // Add new tag observatio into AprilTagInstance and Frame
	    tag_ftr->setAprilTag(tag_instance);
	    tag_instance->addFrameRef(tag_ftr);
	    frame->addAprilTag(tag_ftr);
	    valid_tag_num++;
	} else {
	    tag_ftr->setAprilTag(it->second);
	    it->second->addFrameRef(tag_ftr);
	    frame->addAprilTag(tag_ftr);
	    valid_tag_num++;
	}
	//ROS_INFO("Detected AprilTag %d!", tag_id);
    }
    
    ROS_INFO_STREAM(valid_tag_num << " AprilTags detected!");
        
    //TODO Compute Parallex and design which frame to be marginalized out
    if (window_counter > 0) {
	double parallex = 0.0;
	bool result = computeParallex(window_counter-1, window_counter, parallex);
	// ROS_INFO("parallex    %f    in    %d    window index", parallex, window_counter);
	if (result && parallex > MIN_PARALLAX) {// MIN_PARALLAX = 0.0217
	    marginalization_flag = MARGIN_OLD;
	    ROS_INFO("MARGIN_OLD");
	} else {
	    marginalization_flag = MARGIN_SECOND_NEW;
	    ROS_INFO("MARGIN_SECOND_NEW");
	}
    }
}


    
void Estimator::processIMU(const double& _dt, const Vector3d& _linear_acceleration, const Vector3d& _angular_velocity)
{
    double dt = _dt;
    
    if (first_imu == true) {
	first_imu = false;
	acc_0 = _linear_acceleration;
	gyr_0 = _angular_velocity;
    }
    FramePtr frame = frames[window_counter];
    if (!frame) {
	ROS_WARN("frames[ %d ] has not been instantialize", window_counter);
	return;
    }
    
    // Only happen in initialization stage
    if (!frame->pre_integration) {
	frame->pre_integration.reset(new IntegrationBase{acc_0, gyr_0, frames[window_counter]->Ba, frames[window_counter]->Bg});
    }

    if (window_counter != 0) {
	frame->pre_integration->push_back(dt, _linear_acceleration, _angular_velocity); 
    }
    acc_0 = _linear_acceleration;
    gyr_0 = _angular_velocity;
    
}

void Estimator::slideWindow()
{
    switch (marginalization_flag) {
	case MARGIN_OLD:
	    slideWindowOld();
	    break;
	case MARGIN_SECOND_NEW:
	    slideWindowNew();
	    break;
	default:
	    ROS_WARN("Wrong marginalization flag!");
	    break;
    }

    // TODO Only points and tags invisible to any frame will be deleted. But actually we can delete it when its observations drop to one.
    int n_deleted_point = 0;
    int n_deleted_tag = 0;
    for (auto it = pts_map.begin(), it_next = pts_map.begin(); it != pts_map.end(); it = it_next) {
	it_next++;
	if (it->second->type == Point::TYPE_DELETED) {
	    n_deleted_point++;
	    pts_map.erase(it);
	}
    }  

    for (auto it = tags_map.begin(), it_next = tags_map.begin(); it != tags_map.end(); it = it_next) {
	it_next++;
	if (it->second->type == AprilTagInstance::TYPE_DELETED) {
	    n_deleted_tag++;
	    tags_map.erase(it);
	}
    } 
}


void Estimator::slideWindowOld()
{
    for (unsigned int i = 0; i < WINDOW_SIZE; ++i) {
	frames[i].swap(frames[i+1]);
	frames[i]->setWindowCounter(i);
    }

    frames[WINDOW_SIZE]->clear();
    frames[WINDOW_SIZE].reset(new Frame());  
    
    frames[WINDOW_SIZE]->setWindowCounter(WINDOW_SIZE);
    frames[WINDOW_SIZE]->setPosition(frames[WINDOW_SIZE-1]->P_w);
    frames[WINDOW_SIZE]->setRotation(frames[WINDOW_SIZE-1]->R_w_b);
    frames[WINDOW_SIZE]->setVelocity(frames[WINDOW_SIZE-1]->V_w);
    frames[WINDOW_SIZE]->setBa(frames[WINDOW_SIZE-1]->Ba);
    frames[WINDOW_SIZE]->setBg(frames[WINDOW_SIZE-1]->Bg);
    
    frames[WINDOW_SIZE]->pre_integration.reset(new IntegrationBase{acc_0, gyr_0, frames[WINDOW_SIZE]->Ba, frames[WINDOW_SIZE]->Bg});
}

void Estimator::slideWindowNew()
{
    frames[WINDOW_SIZE-1].swap(frames[WINDOW_SIZE]);
    frames[WINDOW_SIZE-1]->setWindowCounter(WINDOW_SIZE-1);
    frames[WINDOW_SIZE-1]->fusePreintegrationInFront(frames[WINDOW_SIZE]->pre_integration);//charles
    
    frames[WINDOW_SIZE]->clear();
    frames[WINDOW_SIZE].reset(new Frame());    
    
    frames[WINDOW_SIZE]->setWindowCounter(WINDOW_SIZE);
    frames[WINDOW_SIZE]->setPosition(frames[WINDOW_SIZE-1]->P_w);
    frames[WINDOW_SIZE]->setRotation(frames[WINDOW_SIZE-1]->R_w_b);
    frames[WINDOW_SIZE]->setVelocity(frames[WINDOW_SIZE-1]->V_w);
    frames[WINDOW_SIZE]->setBa(frames[WINDOW_SIZE-1]->Ba);
    frames[WINDOW_SIZE]->setBg(frames[WINDOW_SIZE-1]->Bg);
    
    frames[WINDOW_SIZE]->pre_integration.reset(new IntegrationBase{acc_0, gyr_0, frames[WINDOW_SIZE]->Ba, frames[WINDOW_SIZE]->Bg});
}


bool Estimator::computeParallex(int i, int j, double& parallex)
{
    vector<pair<Feature*, Feature*>> match_points;
    getCorrespondingPoints(i, j, match_points);
    if (match_points.size() > 20) {
	double sum_parallex;
	for (auto& m_pt : match_points) {
	    double parallex;
	    computeDistanceByImg(m_pt.first->pt_c, m_pt.second->pt_c, parallex);
	    sum_parallex += parallex;
	}
	parallex = sum_parallex / match_points.size();
	return true;
    } else {
	ROS_WARN("No enough matched points");
	return false;	
    }
}

void Estimator::computeDistanceByImg(const Vector3d& pt_1, const Vector3d& pt_2, double& parallex)
{
    Vector2d dist = pt_1.head<2>() / pt_1.z() - pt_2.head<2>() / pt_2.z();
    parallex =  dist.norm();
}

void Estimator::getCorrespondingPoints(int i, int j, vector< pair< Feature*, Feature* > >& match_points)
{
    vector<pair<Feature*, Feature*>> pt_pairs;
    auto it = frames[i]->fts.begin(); 
    auto it_end = frames[i]->fts.end();
    for (;it != it_end; ++it) {
	Feature* match_ftr = (*it)->point->findFrameRef(frames[j].get());
	// return NULL when not found
	if (!match_ftr) {
	    continue;
	} else {
	    pt_pairs.push_back(make_pair(*it, match_ftr));
	}
    }
    
    match_points = pt_pairs;
}

void Estimator::getCorrespondingTags(int i, int j, vector< std::pair< AprilTagFeature*, AprilTagFeature* > >& match_tags)
{
    vector<pair<AprilTagFeature*,AprilTagFeature*>> tag_pair;
    auto it = frames[i]->apriltag_fts.begin(); auto it_end = frames[i]->apriltag_fts.end();
    for (;it != it_end; ++it) {
	AprilTagFeature* match_tag = (*it)->tag_instance->findFrameRef(frames[j].get());
	if (!match_tag) {
	    tag_pair.push_back(make_pair(*it, match_tag));
	} else {
	    continue;
	}
    }
    match_tags = tag_pair;
}

void Estimator::triangulatePointsInTwoFrames(int idx_a, const Eigen::Matrix<double, 3, 4>& Pose_a, 
					     int idx_b, const Eigen::Matrix<double, 3, 4>& Pose_b)
{
    ROS_ASSERT(idx_a != idx_b);
    
    vector<pair<Feature*, Feature*>> match_fts;
    getCorrespondingPoints(idx_a, idx_b, match_fts);
    
    for (auto& ftr_pair : match_fts) {
	if (ftr_pair.first->point->type == Point::TYPE_TRIANGULATED)
	    continue;
	
	Vector3d pt_w;
	Vector2d pt_a = ftr_pair.first->pt_c.head<2>() / ftr_pair.first->pt_c.z();
	Vector2d pt_b = ftr_pair.second->pt_c.head<2>() / ftr_pair.second->pt_c.z(); 
	triangulatePoint(Pose_a, Pose_b, pt_a, pt_b, pt_w);
	
	Vector3d pt_c;
	if (!(projectW2C(Pose_a.block<3, 3>(0, 0), Pose_a.block<3, 1>(0, 3), pt_w, pt_c) && projectW2C(Pose_b.block<3, 3>(0, 0), Pose_b.block<3, 1>(0, 3), pt_w, pt_c))) {
	    continue;
	}
	
	ftr_pair.first->point->setPosition(pt_w);
    }
}

void Estimator::triangulatePoint(const Matrix< double, int(3), int(4) >& _P_a, 
				 const Matrix< double, int(3), int(4) >& _P_b, 
				 const Vector2d& _pt_a, const Vector2d& _pt_b, Vector3d& pt_w)
{
    Matrix4d design_matrix = Matrix4d::Zero();
    design_matrix.row(0) = _pt_a[0] * _P_a.row(2) - _P_a.row(0);
    design_matrix.row(1) = _pt_a[1] * _P_a.row(2) - _P_a.row(1);
    design_matrix.row(2) = _pt_b[0] * _P_b.row(2) - _P_b.row(0);
    design_matrix.row(3) = _pt_b[1] * _P_b.row(2) - _P_b.row(1);
    Vector4d triangulated_point;
    triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    pt_w(0) = triangulated_point(0) / triangulated_point(3);
    pt_w(1) = triangulated_point(1) / triangulated_point(3); 
    pt_w(2) = triangulated_point(2) / triangulated_point(3);
}


bool Estimator::solveGlobalRT(int idx, Matrix3d& R, Vector3d& t, int flag, bool use_initial)// R_c_w, t_c
{
    int pt_num = frames[idx]->fts.size();
    int tag_num = frames[idx]->apriltag_fts.size();
    
    if (flag == 1 && tag_num < 1)
	return false;
	
    if (flag == 2 && pt_num < 10)
	return false;
    
    if (flag == 0 && tag_num < 1 && pt_num < 10)
	return false;
    
    vector<Vector3d> pts;
    vector<Vector2d> pxs;

    if (flag == 0 || flag == 2) {
	for (auto ftr : frames[idx]->fts) {
	    if (ftr->point->type != Point::TYPE_TRIANGULATED)
		continue;
	    pts.push_back(ftr->point->P_w);
	    pxs.push_back(ftr->pt_c.head<2>());
	}	
    }
       
    if (flag == 0 || flag == 1) {
	for (auto tag : frames[idx]->apriltag_fts) {
	    vector<Vector2d> tag_pxs;
	    tag_pxs.clear();
// 	    assert(tag->tag_fts.size() == 4);
	    for (auto corner : tag->tag_fts) {
		tag_pxs.push_back(corner->pt_c.head<2>());
	    }
	    
	    pts.insert(pts.end(), tag->tag_instance->pts_w.begin(), tag->tag_instance->pts_w.end());
	    pxs.insert(pxs.end(), tag_pxs.begin(), tag_pxs.end());
	}	
    } 

    cv::Matx33d cam = cv::Matx33d::eye(); cv::Vec4d dist(0, 0, 0, 0);
    
    bool result = solveRTByPnP(pts, pxs, R, t, cam, dist, use_initial); // true: use initial values
    if (!result)
	return false;

    return true;
}

bool Estimator::solvePnPByLeds(int idx, Matrix3d& R, Vector3d& t, bool use_initial)// R_c_w, t_c
{
    int tag_num = frames[idx]->apriltag_fts.size();

    vector<Vector3d> pts;
    vector<Vector2d> pxs;

    if (tag_num < 4)
	return false;    
    
    for (auto tag : frames[idx]->apriltag_fts) {
	pts.push_back(tag->tag_instance->pt_center);
	pxs.push_back(tag->ftr_center.head<2>());
    }	

    cv::Matx33d cam = cv::Matx33d::eye(); cv::Vec4d dist(0, 0, 0, 0);
  
    bool result = solveRTByPnP(pts, pxs, R, t, cam, dist, use_initial); // true: use initial values
    if (!result)
	return false;

    return true;
}

bool Estimator::solveRTByPnP(const vector<Vector3d>& _pts, const vector<Vector2d>& _pxs, 
			     Matrix3d& R, Vector3d& t, cv::Matx33d _cam , cv::Vec4d _dist, bool use_initial)
{
    vector<cv::Point3d> pts;
    vector<cv::Point2d> pxs;
    for (unsigned int i = 0; i < _pts.size(); ++i) {
	pts.push_back(cv::Point3d(_pts[i].x(), 
				      _pts[i].y(), 
				      _pts[i].z()));
	pxs.push_back(cv::Point2d(_pxs[i].x(), _pxs[i].y()));	
    }
    if (pts.size() < 4)
	return false;
    
    cv::Mat rvec, tvec, tmp_r;
    
    // Initial values of rvec and tvec
    cv::eigen2cv(R, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(t, tvec);
    
    bool pnp_result = cv::solvePnP(pts, pxs, _cam, _dist, rvec, tvec, use_initial);
    if (!pnp_result)
	return false;
    cv::Matx33d r;
    cv::Rodrigues(rvec, r);
    R << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);
    t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    return true;
}




bool Estimator::getTriangulateCandidate(const vector< int >& _tag_index, int& idx)
{
    int nearest_idx = _tag_index.back();
    for (auto& i : _tag_index) {
	if (i == nearest_idx)
	    break;
	double parallex;
	bool result = computeParallex(i, nearest_idx, parallex);
	if (result && (parallex > MIN_PARALLAX)) {
	    idx = i;
	    // ROS_INFO("Find proper initial AprilTag frame...");
	    return true;
	} 
    }
    
    ROS_WARN("Fail to triangulate AprilTag frame!");
    return false;
}


bool Estimator::visualInitialAlign()
{	
    
    if(!solveGyroscopeBias()) {
	return false;
    }

    //ROS_INFO_STREAM("******Bas : " << std::endl << frames[window_counter]->Bg.transpose());

    if(!linearAlignment())
	return false;
    
    return true;
}

bool Estimator::solveGyroscopeBias()
{
    Eigen::Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    delta_bg.setZero();
    A.setZero();
    b.setZero();
    
    for (unsigned int i = 0; i < WINDOW_SIZE; ++i) {
	Eigen::Matrix3d J_bg;
	J_bg.setZero();
	J_bg = frames[i]->pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
	
	Vector3d residual;
	residual.setZero();
	Eigen::Quaterniond q_ij(frames[i]->R_w_b.transpose() * frames[i+1]->R_w_b); 
	residual = 2 * (frames[i]->pre_integration->delta_q.inverse() * q_ij).vec();
	
	A += J_bg.transpose() * J_bg;
	b += J_bg.transpose() * residual;
    }
    A = A * 1000;
    b = b * 1000;
    delta_bg = A.ldlt().solve(b);
    
    if (std::isnan(delta_bg(0)) || std::isnan(delta_bg(1)) || std::isnan(delta_bg(2))) {
	ROS_WARN("g is nan");
	return false;
    }     
    
    ROS_WARN_STREAM("Gyroscope bias initial calibration: " << delta_bg.transpose());
    
    for (unsigned int i = 0; i <= WINDOW_SIZE; ++i) {
	frames[i]->setBa(Vector3d::Zero());
	
	frames[i]->setBg(delta_bg);
	frames[i]->pre_integration->repropagate(Vector3d::Zero(), frames[i]->Bg);
    }
    
    return true;
}

bool Estimator::linearAlignment()
{
    Vector3d estimate_g;
    VectorXd x;
    x.setZero();
    
    int n_state = (WINDOW_SIZE + 1) * 3 + 3;
    Eigen::MatrixXd A(n_state, n_state);
    VectorXd b(n_state);
    A.setZero();
    b.setZero();
    for (unsigned int i = 0; i < WINDOW_SIZE; ++i) {
	Eigen::MatrixXd H(6, 9);
	VectorXd z(6);
	H.setZero();
	z.setZero();
	
	double dt = frames[i+1]->pre_integration->sum_dt;

        H.block<3, 3>(0, 0) = -SCALE_TO_METRIC * dt * Matrix3d::Identity();
        H.block<3, 3>(0, 6) = frames[i]->R_w_b.transpose() * dt * dt / 2 * Matrix3d::Identity();
        z.block<3, 1>(0, 0) = frames[i+1]->pre_integration->delta_p + frames[i]->R_w_b.transpose() * frames[i+1]->R_w_b * tic - tic - SCALE_TO_METRIC * frames[i]->R_w_b.transpose() * (frames[i+1]->P_w - frames[i]->P_w);
	
        H.block<3, 3>(3, 0) = -SCALE_TO_METRIC * Matrix3d::Identity();
        H.block<3, 3>(3, 3) = SCALE_TO_METRIC * frames[i]->R_w_b.transpose() * frames[i+1]->R_w_b;
        H.block<3, 3>(3, 6) = frames[i]->R_w_b.transpose() * dt * Matrix3d::Identity();
        z.block<3, 1>(3, 0) = frames[i+1]->pre_integration->delta_v;

        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        cov_inv.setIdentity();
	
	MatrixXd tmp_A = H.transpose() * cov_inv * H;
	VectorXd tmp_b = H.transpose() * cov_inv * z;
	
	A.block<6, 6>(i*3, i*3) += tmp_A.topLeftCorner<6, 6>();
	b.segment<6>(i*3) += tmp_b.head<6>();
	
	A.bottomRightCorner<3, 3>() += tmp_A.bottomRightCorner<3, 3>();
	b.tail<3>() += tmp_b.tail<3>();
	
	A.block<6, 3>(i*3, n_state - 3) += tmp_A.topRightCorner<6, 3>();
	A.block<3, 6>(n_state - 3, i*3) += tmp_A.bottomLeftCorner<3, 6>();
    }
  
    A = A * 1000;
    b = b * 1000;
    x = A.ldlt().solve(b);

    estimate_g = x.tail<3>();
    if (std::isnan(estimate_g(0)) || std::isnan(estimate_g(1)) || std::isnan(estimate_g(2))) {
	ROS_WARN("g is nan");
	return false;
    } 
    
    if(fabs(estimate_g.norm() - G.norm()) > 1.0) {
	ROS_WARN("g is wrong");
	return false;
    } 
    
ROS_WARN_STREAM("Gravity raw     " << estimate_g.norm() << " " << estimate_g.transpose());

    VectorXd estimate_x;
    RefineGravity(estimate_g, estimate_x);
    g = estimate_g;
    
    ROS_WARN_STREAM("Gravity refine     " << g.norm() << " " << g.transpose());
    
    // Set initial velocity
    for (unsigned int i = 0; i <= WINDOW_SIZE; ++i) {
	frames[i]->setVelocity( frames[i]->R_w_b * estimate_x.segment<3>(i*3));
    }
    // Set constant gravity
    
    return true;
}

Eigen::MatrixXd TangentBasis(Vector3d& g0)
{
    Vector3d b, c;
    Vector3d a = g0.normalized();
    Vector3d tmp(0, 0, 1);
    if(a == tmp)// if g0 does point to z axis
        tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}

void Estimator::RefineGravity(Vector3d& _g, VectorXd& _x)
{
    Vector3d g0 = _g.normalized() * G.norm();
    Vector3d lx, ly;
    int n_state = (WINDOW_SIZE + 1) * 3 + 2;
      
    Eigen::MatrixXd A(n_state, n_state);
    VectorXd b(n_state);
    A.setZero();
    b.setZero();

    for (int k = 0; k < 4; ++k) {
	MatrixXd lxly(3, 2);
	lxly = TangentBasis(g0);
	
	for (unsigned int i = 0; i < WINDOW_SIZE; ++i) {
	    Eigen::MatrixXd H(6, 8);
	    VectorXd z(6);
	    H.setZero();
	    z.setZero();

	    double dt = frames[i+1]->pre_integration->sum_dt;

	    H.block<3, 3>(0, 0) = -SCALE_TO_METRIC * dt * Matrix3d::Identity();
	    H.block<3, 2>(0, 6) = frames[i]->R_w_b.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
	    z.block<3, 1>(0, 0) = frames[i+1]->pre_integration->delta_p + frames[i]->R_w_b.transpose() * frames[i+1]->R_w_b * tic - tic - frames[i]->R_w_b.transpose() * dt * dt / 2 * g0  - SCALE_TO_METRIC * frames[i]->R_w_b.transpose() * (frames[i+1]->P_w - frames[i]->P_w);

	    H.block<3, 3>(3, 0) = -SCALE_TO_METRIC * Matrix3d::Identity();
	    H.block<3, 3>(3, 3) = SCALE_TO_METRIC * frames[i]->R_w_b.transpose() * frames[i+1]->R_w_b;
	    H.block<3, 2>(3, 6) = frames[i]->R_w_b.transpose() * dt * Matrix3d::Identity() * lxly;
	    z.block<3, 1>(3, 0) = frames[i+1]->pre_integration->delta_v - frames[i]->R_w_b.transpose() * dt * Matrix3d::Identity() * g0;

	    Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
	    cov_inv.setIdentity();
	    
	    MatrixXd tmp_A = H.transpose() * cov_inv * H;
	    VectorXd tmp_b = H.transpose() * cov_inv * z;

	    A.block<6, 6>(i*3, i*3) += tmp_A.topLeftCorner<6, 6>();
	    b.segment<6>(i*3) += tmp_b.head<6>();
	    
	    A.bottomRightCorner<2, 2>() += tmp_A.bottomRightCorner<2, 2>();
	    b.tail<2>() += tmp_b.tail<2>();

	    A.block<6, 2>(i*3, n_state - 2) += tmp_A.topRightCorner<6, 2>();
	    A.block<2, 6>(n_state - 2, i*3) += tmp_A.bottomLeftCorner<2, 6>();	    
	    
	}

	A = A * 1000.0;
	b = b * 1000.0;
	_x = A.ldlt().solve(b);

	VectorXd dg = _x.tail<2>();
	g0 = (g0 + lxly * dg).normalized() * G.norm();	
    } 
ROS_INFO("D");
    _g = g0;
}


bool Estimator::construct(int left_idx, int right_idx)
{     
//     int n_triang_start = 0;
//     for (auto& pt_m : pts_map) {
// 	if (pt_m.second->type == Point::TYPE_TRIANGULATED)
// 	    n_triang_start++;
//     }
    //ROS_INFO("Start: %d Points have been triangulated", n_triang_start);
    
    Eigen::Matrix3d R_c[WINDOW_SIZE + 1];
    Eigen::Quaterniond Quat_c[WINDOW_SIZE + 1];
    Vector3d T_c[WINDOW_SIZE + 1];
    Eigen::Matrix<double, 3, 4> Pose[WINDOW_SIZE + 1];
    double rotation[WINDOW_SIZE + 1][4];
    double translation[WINDOW_SIZE + 1][3];
    
    Matrix3d R_c_w;  
    Vector3d P_c;
// ROS_INFO("A      1");   
    if (!solveGlobalRT(left_idx, R_c_w, P_c, 1, false))	// Global position and rotation
	return false;
    R_c[left_idx] = R_c_w;
    Quat_c[left_idx] = R_c_w;
    T_c[left_idx] = P_c;
    Pose[left_idx].block<3, 3>(0, 0) = R_c[left_idx];
    Pose[left_idx].block<3, 1>(0, 3) = T_c[left_idx];
// ROS_INFO("A      1      1");   
    if(!solveGlobalRT(right_idx, R_c_w, P_c, 1, false))
	return false;
    R_c[right_idx] = R_c_w;
    Quat_c[right_idx] = R_c_w;
    T_c[right_idx] = P_c;
    Pose[right_idx].block<3, 3>(0, 0) = R_c[right_idx];
    Pose[right_idx].block<3, 1>(0, 3) = T_c[right_idx];  
// ROS_INFO("A      2");    
    /*
     * 1. Construct: left_idx and right_idx
     */    
    triangulatePointsInTwoFrames(left_idx, Pose[left_idx], right_idx, Pose[right_idx]);

// ************************************************************************************** 测试solvePnP准不准,结果:准   
//  frames[left_idx]   
//     Vector2d pts_2 = frames[left_idx]->apriltag_fts.back()->tag_fts[0]->pt_c.head<2>();
//     Vector3d pts_3 = frames[left_idx]->apriltag_fts.back()->tag_instance->pts_w[0];
// 
//     Vector2d pts_2_hat = (R_c[left_idx] * pts_3 + T_c[left_idx]).head<2>();
//    
//     for (unsigned int i = 0; i < 4; ++i) {
// 	Vector2d pts_2 = frames[left_idx]->apriltag_fts.back()->tag_fts[i]->pt_c.head<2>();
// 	Vector3d pts_3 = frames[left_idx]->apriltag_fts.back()->tag_instance->pts_w[i];
// 	Vector3d pts_2_s = R_c[left_idx] * pts_3 + T_c[left_idx];
// 	Vector2d pts_2_hat = pts_2_s.head<2>() / pts_2_s.z();
// 	
// 	ROS_INFO_STREAM("Tag " << frames[left_idx]->apriltag_fts.back()->tag_instance->id << "Projection****** TRUE: " << pts_2.transpose() << std::endl << 
// 	    "--------------------------------------------ESTIMATED: " << pts_2_hat.transpose() );
//     }  
    
    
// ************************************************************************************** 测试triangulatePoint准不准, 结果: 准

    
/*       
    // triangulatePointMap
    for (unsigned int i = 0; i < 4; ++i) {
	
	Eigen::MatrixXd svd_A(2 * 2, 4);
	int svd_idx = 0; 
	
	Vector3d f = frames[left_idx]->apriltag_fts.back()->tag_fts[i]->pt_c;
	Pose[left_idx];
	svd_A.row(svd_idx++) = f[0] * Pose[left_idx].row(2) - f[2] * Pose[left_idx].row(0);	
	svd_A.row(svd_idx++) = f[1] * Pose[left_idx].row(2) - f[2] * Pose[left_idx].row(1);	
	
	f = frames[right_idx]->apriltag_fts.back()->tag_fts[i]->pt_c;
	Pose[right_idx];

	svd_A.row(svd_idx++) = f[0] * Pose[right_idx].row(2) - f[2] * Pose[right_idx].row(0);	
	svd_A.row(svd_idx++) = f[1] * Pose[right_idx].row(2) - f[2] * Pose[right_idx].row(1);
	
	ROS_ASSERT(svd_idx == svd_A.rows());
	Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
	Vector3d P_w(svd_V[0]/svd_V[3], svd_V[1]/svd_V[3], svd_V[2]/svd_V[3]);
	
	ROS_INFO_STREAM("Tag " << frames[left_idx]->apriltag_fts.back()->tag_instance->id << " Pt_w****** TRUE: " << frames[left_idx]->apriltag_fts.back()->tag_instance->pts_w[i].transpose() << std::endl << 
	    "------------------------------------------------------ESTIMATED: " << P_w.transpose() );	
    }	
	
    
  */  
    
    
// ************************************************************************************** 测试solvePnP准不准,结果:准   
/*    
    for (unsigned int i = 0; i < 4; ++i) {
	Vector2d pt_i = frames[left_idx]->apriltag_fts.back()->tag_fts[i]->pt_c.head<2>();
	Vector2d pt_j = frames[right_idx]->apriltag_fts.back()->tag_fts[i]->pt_c.head<2>();
	Vector3d pt_w;
	triangulatePoint(Pose[left_idx], Pose[right_idx], pt_i, pt_j, pt_w);
	ROS_INFO_STREAM("Tag " << frames[left_idx]->apriltag_fts.back()->tag_instance->id << " Pt_w****** TRUE: " << frames[left_idx]->apriltag_fts.back()->tag_instance->pts_w[i].transpose() << std::endl << 
	    "------------------------------------------------------ESTIMATED: " << pt_w.transpose() );
    }
    */
    
    
    
    
// **************************************************************************************     
    /*
     * 2. Construct: idx+1 -> right_idx
     */
    for (unsigned int idx = left_idx+1; idx < right_idx; ++idx) {
	Matrix3d R_initial = R_c[idx - 1];
	Vector3d P_initial = T_c[idx - 1];	
	if (!solveGlobalRT(idx, R_initial, P_initial, 0, true))
	    return false;
	R_c[idx] = R_initial;
	Quat_c[idx] = R_initial;
	T_c[idx] = P_initial;
	Pose[idx].block<3, 3>(0, 0) = R_c[idx];
	Pose[idx].block<3, 1>(0, 3) = T_c[idx];
	triangulatePointsInTwoFrames(idx, Pose[idx], right_idx, Pose[right_idx]);
    }
// ROS_INFO("A      3");   
    /*
     * 3. Construct: right_idx-1 -> left_idx
     */
    for (unsigned int idx = left_idx+1; idx < right_idx; ++idx) {
	triangulatePointsInTwoFrames(left_idx, Pose[left_idx], idx, Pose[idx]);
    }

    /*
     * 4. Construct: right_idx+1 -> WINDOW_SIZE+1
     */
    for (unsigned int idx = right_idx+1; idx <= WINDOW_SIZE; ++idx) {
	Matrix3d R_initial = R_c[idx - 1];
	Vector3d P_initial = T_c[idx - 1];
	if (!solveGlobalRT(idx, R_initial, P_initial, 0, true))
	    return false;
	R_c[idx] = R_initial;
	Quat_c[idx] = R_initial;
	T_c[idx] = P_initial;
	Pose[idx].block<3, 3>(0, 0) = R_c[idx];
	Pose[idx].block<3, 1>(0, 3) = T_c[idx];
	triangulatePointsInTwoFrames(right_idx, Pose[right_idx], idx, Pose[idx]);
    }
// ROS_INFO("A      4");   
     /*
     * 5. Construct: left_idx-1 -> 0
     */
    if (left_idx > 0) {
	for (int idx = left_idx-1; idx >= 0; --idx) {
	    // ROS_INFO("%d idx", idx);
	    Matrix3d R_initial = R_c[idx + 1];
	    Vector3d P_initial = T_c[idx + 1];
	    if (!solveGlobalRT(idx, R_initial, P_initial, 0, true))
		return false;
	    R_c[idx] = R_initial;
	    Quat_c[idx] = R_initial;
	    T_c[idx] = P_initial;
	    Pose[idx].block<3, 3>(0, 0) = R_c[idx];
	    Pose[idx].block<3, 1>(0, 3) = T_c[idx];
	    triangulatePointsInTwoFrames(idx, Pose[idx], left_idx, Pose[left_idx]);
	}
		
    }
// ROS_INFO("A      5");    
    // 6. Triangulate all other points
//     for (auto& pt_m : pts_map) {
// 	Point* pt = pt_m.second;
// 	if (pt->type == Point::TYPE_TRIANGULATED)
// 	    continue;
// 	
// 	if (pt->obs.size() < 2) 
// 	    continue;
// 	
// 	int frame_a = pt->obs.front()->frame->id;
// 	Vector2d pt_a = pt->obs.front()->pt_c.head<2>();// z = 1
// 	
// 	int frame_b = pt->obs.back()->frame->id;
// 	Vector2d pt_b = pt->obs.back()->pt_c.head<2>();
// 	
// 	Vector3d pt_w;
// 	triangulatePoint(Pose[frame_a], Pose[frame_b], pt_a, pt_b, pt_w);
// 	pt->setPosition(pt_w);	
//     }
    
    int n_triang = 0;
    for (auto& pt_m : pts_map) {
	if (pt_m.second->type == Point::TYPE_TRIANGULATED)
	    n_triang++;
    }
    ROS_INFO("%d Points have been triangulated", n_triang);
    
    
    ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
    
    for (unsigned int i = 0; i < WINDOW_SIZE+1; ++i) {
	translation[i][0] = T_c[i].x();
	translation[i][1] = T_c[i].y();
	translation[i][2] = T_c[i].z();
	rotation[i][0] = Quat_c[i].w();
	rotation[i][1] = Quat_c[i].x();
	rotation[i][2] = Quat_c[i].y();
	rotation[i][3] = Quat_c[i].z();
	problem.AddParameterBlock(translation[i], 3);
	problem.AddParameterBlock(rotation[i], 4, local_parameterization);

	if (i == left_idx || i == right_idx) {
	    problem.SetParameterBlockConstant(rotation[i]);
	    problem.SetParameterBlockConstant(translation[i]);
	}
	    
    }
    
    for (auto& tag_m : tags_map) {
	AprilTagInstance* tag_instance = tag_m.second;
	for (unsigned int i = 0; i < 4; ++i) {
	    problem.AddParameterBlock(tag_instance->position[i], 3);	
	    problem.SetParameterBlockConstant(tag_instance->position[i]);
	}
    }     

    for (auto& tag_m : tags_map) {
	AprilTagInstance* tag_instance = tag_m.second;
	ROS_INFO("Number of tag observasion:  %d", tag_instance->tag_obs.size());
	for (auto& tag_ftr : tag_instance->tag_obs) {
	    for (unsigned int i = 0; i < 4; ++i) {
		
		int idx = tag_ftr->frame->window_counter;
		Vector3d corner = tag_ftr->tag_fts[i]->pt_c;
		ceres::CostFunction* cost_function = ReprojectionError3D::Create(corner.x(), 
										 corner.y());
		problem.AddResidualBlock(cost_function, NULL, rotation[idx], translation[idx], tag_instance->position[i]);		
	    }
	    
	}
    }    
    
    for (auto it = pts_map.begin(); it != pts_map.end(); ++it) {
	Point* pt = (*it).second;
	if (pt->type != Point::TYPE_TRIANGULATED)
	    continue;
	
	auto it_ftr = pt->obs.begin();
	auto it_end = pt->obs.end();
	for (;it_ftr != it_end; ++it_ftr) {
	    int idx = (*it_ftr)->frame->window_counter;
	    ceres::CostFunction* cost_function = ReprojectionError3D::Create((*it_ftr)->pt_c.x(), 
									     (*it_ftr)->pt_c.y());
	    problem.AddResidualBlock(cost_function, NULL, rotation[idx], translation[idx], pt->position);
	}

    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_solver_time_in_seconds = 0.2;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03)
    {
	ROS_INFO("Ceres converge!");
    }
    else
    {
	ROS_INFO("Ceres does not converge...");
	return false;
    }    
    
    for (unsigned int i = 0; i < WINDOW_SIZE+1; ++i) {
	Quat_c[i].w() = rotation[i][0];
	Quat_c[i].x() = rotation[i][1]; 
	Quat_c[i].y() = rotation[i][2]; 
	Quat_c[i].z() = rotation[i][3]; 
	
	Eigen::Quaterniond Quat_w;
	Vector3d T_w;
	Quat_w = Quat_c[i].normalized().inverse();
	T_w = -1 * (Quat_w * Vector3d(translation[i][0], translation[i][1], translation[i][2]));
	
	frames[i]->setRotation(Quat_w.toRotationMatrix() * ric.transpose());
	frames[i]->setPosition(T_w);
    }
/*
    for (auto& pt_m : pts_map) {
	Point* pt = pt_m.second;
	if (pt->type == Point::TYPE_TRIANGULATED) {
	    // Points have been triangulated
	    pt->setPositionByPara();
// 	    pt->type = Point::TYPE_INIT;
	}
    }*/
/*    
n_triang = 0;
for (auto& pt_m : pts_map) {
    if (pt_m.second->type == Point::TYPE_TRIANGULATED)
	n_triang++;
}
ROS_INFO("%d Points have been triangulated", n_triang);  */  

//     triangulatePointMap();
    return true;
}


bool Estimator::consultDictionary(const int& _id, Matrix3d& R, Vector3d& t, double& size)
{
    auto it = global_tags_dic.find(_id);//GLOBAL_TAGS_DIC
    if (it == global_tags_dic.end()) {
	return false;
    } else {
	t = it->second.second.first;
	R = it->second.second.second;
	size = it->second.first;
	return true;
    }
}

void Estimator::loadDictionary()
{
    {
	vector<Point*> pts;
	int tag_id = 0;
	double tag_size = TAG_SIZE;
	Vector3d eigen_t(0, 0, 0);
	Matrix3d eigen_R = Eigen::MatrixXd::Identity(3, 3);
	global_tags_dic[tag_id] = make_pair(tag_size, make_pair(eigen_t, eigen_R));	
    }
    
    {
	vector<Point*> pts;
	int tag_id = 1;
	double tag_size = TAG_SIZE;
	Vector3d eigen_t(14.2, 0, 0);
	Matrix3d eigen_R = Eigen::MatrixXd::Identity(3, 3);
	global_tags_dic[tag_id] = make_pair(tag_size, make_pair(eigen_t, eigen_R));	
    }
    
    {
	vector<Point*> pts;
	int tag_id = 2;
	double tag_size = TAG_SIZE;
	Vector3d eigen_t(14.2, -21.35, 0);
	Matrix3d eigen_R = Eigen::MatrixXd::Identity(3, 3);
	global_tags_dic[tag_id] = make_pair(tag_size, make_pair(eigen_t, eigen_R));	
    }

    {
	vector<Point*> pts;
	int tag_id = 3;
	double tag_size = TAG_SIZE;
	Vector3d eigen_t(0, -21.35, 0);
	Matrix3d eigen_R = Eigen::MatrixXd::Identity(3, 3);
	global_tags_dic[tag_id] = make_pair(tag_size, make_pair(eigen_t, eigen_R));	
    }
}


bool Estimator::projectW2C(const Matrix3d& _R_c_w, const Vector3d& _t_c, const Vector3d& _P_w, Vector3d& P_c) 
{
    Vector3d pt_c = _R_c_w * _P_w + _t_c;
    double inv_dep = 1.0 / pt_c.z();
    if (inv_dep < 0.02)
	return false;
    
    P_c = pt_c * inv_dep;
    return true;
}

bool Estimator::projectW2C(const Matrix3d& _R_w_b, const Vector3d& _t_w, const Matrix3d& _r_b_c, const Vector3d& _t_b_c, const Vector3d& _P_w, Vector3d& P_c)
{
    Vector3d P_c_s = _r_b_c.transpose() * (_R_w_b.transpose() * (_P_w - _t_w) - _t_b_c);
    double inv_dep = 1.0 / P_c_s.z();
    if (inv_dep < 0.02)
	return false;
    
    P_c = P_c_s * inv_dep;
    return true;
}

void Estimator::setParameter()
{
    ric = RIC[0];
    tic = TIC[0];	
    
    para_Ex_Pose[0] = tic.x();
    para_Ex_Pose[1] = tic.y();
    para_Ex_Pose[2] = tic.z();
    Eigen::Quaterniond q{ric};
    para_Ex_Pose[3] = q.x();
    para_Ex_Pose[4] = q.y();
    para_Ex_Pose[5] = q.z();
    para_Ex_Pose[6] = q.w();

    loadDictionary();
    
    ReprojectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ConstPointFactor::const_sqrt_info = 1.5 * Matrix2d::Identity();
    PointFactor::sqrt_info = 0.5 * Matrix2d::Identity();
}


bool Estimator::saveData(ofstream* outfile, const int& _index, const Matrix3d& _R, const Vector3d& _t, string _text)
{
    if ((*outfile).is_open()) {	
	Vector3d ypr = Utility::R2ypr(_R);
	try {
	    (*outfile) << _index << "   " << _t.transpose() << "       " << ypr.transpose() << "\n";
	} catch (exception& e) {
	    ROS_ERROR("saving data fails...");
	}    
    } else {
	ROS_ERROR("saving data fails...");
	return false;
    }
    return true;
}

} // end basic



/*   


 
    for (auto& pt_m : pts_map) {
	Point* pt = pt_m.second;
	if (pt->obs.size() < 2 || pt->type != Point::TYPE_TRIANGULATED)
	    continue;

	problem.AddParameterBlock(pt->position, 3);	
	//problem.SetParameterBlockConstant(pt->position);	
    } 

 
    for (auto& pt_m : pts_map) {
	Point* pt = pt_m.second;
	if (pt->obs.size() < 2 || pt->type != Point::TYPE_TRIANGULATED)
	    continue;

	for (auto& ftr : pt->obs) {
	    PointFactor* point_factor = new PointFactor(ftr->pt_c);
	    problem.AddResidualBlock(point_factor, loss_function, ftr->frame->para_Pose, para_Ex_Pose, pt->position);
	}
    }    

    // Points
    for (auto& pt_m : pts_map) {
	Point* pt = pt_m.second;
	if (pt->type == Point::TYPE_TRIANGULATED) {
	    pt->setPositionByPara();
	}
    }  
    
*/