cmake_minimum_required(VERSION 2.8.3)
project(tag_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    sensor_msgs
    cv_bridge
    camera_model
    image_transport
    tf
    geometry_msgs
    )

catkin_package()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system )

include_directories(
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
	)

link_directories(
    ${catkin_LIB_DIRS}  lib)


include_directories(/home/charles/code/apriltag_c)
include_directories(/home/charles/code/apriltag_c/common)

# add_dependencies(apriltag_detector apriltags_ros_generate_messages_cpp)

add_executable(tag_tracker
	src/tag_tracker_node.cpp
	src/parameters.cpp
	src/apriltag_manager.cpp
	src/point.cpp
	src/tag_detector.cpp
	src/FeatureTracker.cpp
	)
target_link_libraries(tag_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} libapriltag.so )
