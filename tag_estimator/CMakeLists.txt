cmake_minimum_required(VERSION 2.8.3)
project(tag_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
)

find_package(OpenCV REQUIRED)

find_package(Ceres REQUIRED)

find_package(Eigen3)
include_directories(
	${catkin_INCLUDE_DIRS} 
	${CERES_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_executable(tag_estimator
	src/tag_estimator_node.cpp
	src/parameters.cpp
	src/utility/utility.cpp
	src/aprilTag.cpp
	src/estimator.cpp
	src/frame.cpp
	src/point.cpp
	src/factor/integration_base.h
	src/factor/reprojection_factor.cpp
	src/factor/point_factor.cpp
	src/factor/const_point_factor.cpp
	src/factor/pose_local_parameterization.cpp
)
target_link_libraries(tag_estimator ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 
