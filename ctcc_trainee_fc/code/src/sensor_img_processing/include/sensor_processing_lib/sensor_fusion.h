/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 19/04/2018
 *
 */

// Include guard
#ifndef sensor_processing_img_H
#define sensor_processing_img_H

// Includes
#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_img_processing/ObjectArray.h>
#include "yolo_detector.h"
#include "cxxopts.hpp"

// Namespaces
namespace sensor_img_processing{

using namespace sensor_msgs;
using namespace Eigen;

// Parameter handler
struct Parameters{

    int image_width;
	int image_height;
	bool is_gpu;
	float conf_thres;
	float iou_thres;

};


class SensorFusion{

public:

	// Default constructor
	SensorFusion(ros::NodeHandle nh, ros::NodeHandle private_nh);

	// Virtual destructor
	virtual ~SensorFusion();

	// Processes raw camera image
	// and publishes the output grid message
	virtual void process(
		const Image::ConstPtr & image
	);


private:

	// Node handle
	ros::NodeHandle nh_, private_nh_;

	// Class members
	Parameters params_;
	ObjectArray object_array_;

	// Semantic variables
	std::vector<std::string> SEMANTIC_NAMES;
	std::map<int, int> SEMANTIC_COLOR_TO_CLASS;
	std::map<std::string, int> SEMANTIC_NAME_TO_INDEX;
	MatrixXi SEMANTIC_CLASS_TO_COLOR;

	std::shared_ptr<Detector> yolo_detector_ptr_;
    std::vector<std::string> class_names_;

	int time_frame_;

	// Publisher
	ros::Publisher yolo_detection_pub_;
	ros::Publisher object_array_pub_;

	// Subscriber
	// Subscriber<Image> image_sub_;
	ros::Subscriber image_sub_;

	// Class functions
	void InitParam();

	void processImage(const Image::ConstPtr & image);

    std::vector<std::string> LoadNames(const std::string& path);

    void viz_yolo_detections(cv::Mat& img, const std::vector<std::vector<Detection>>& detections);
};

} // namespace sensor_processing_img

#endif // sensor_processing_img_H
