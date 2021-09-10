/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 19/04/2018
 *
 */

#include <sensor_processing_lib/sensor_fusion.h>
#include <ros/package.h>
#include <chrono>

namespace sensor_img_processing{

/******************************************************************************/

SensorFusion::SensorFusion(ros::NodeHandle nh, ros::NodeHandle private_nh):
	nh_(nh),
	private_nh_(private_nh){

	// Define image parameters
	private_nh_.param("image/width", params_.image_width,
		params_.image_width);
	private_nh_.param("image/height", params_.image_height,
		params_.image_height);
	private_nh_.param("image/is_gpu", params_.is_gpu,
		params_.is_gpu);
	private_nh_.param("image/conf_thres", params_.conf_thres,
		params_.conf_thres);
	private_nh_.param("image/iou_thres", params_.iou_thres,
		params_.iou_thres);

	// Print parameters
	ROS_INFO_STREAM("is_gpu " << params_.is_gpu);

	// class private variables init
	InitParam();

	// Define Publisher 
	yolo_detection_pub_ = nh_.advertise<Image>(
		"/sensor/image/yolo_detections", 2);
	object_array_pub_ = nh.advertise<ObjectArray>(
	    "/detection/objects", 2);

	// Define Subscriber
    image_sub_ = nh_.subscribe("/ue09_pylon02/image_raw", 2, &SensorFusion::process, this);

    // set device type - CPU/GPU
    torch::DeviceType device_type;
    if (torch::cuda::is_available() && params_.is_gpu) {
        device_type = torch::kCUDA;
    } else {
        device_type = torch::kCPU;
    }
    // load class names from dataset for visualization
    std::string package_path = ros::package::getPath("sensor_img_processing");
    std::string class_file_path = package_path + "/weight/coco.names";
    std::cout << "class path:" << class_file_path << std::endl;
    class_names_ = LoadNames(class_file_path);

    // load network
    std::string weights_path = package_path + "/weight/yolov5l.torchscript.pt";
    std::cout << "weight path: " << weights_path << std::endl;
    yolo_detector_ptr_ = std::make_shared<Detector>(weights_path, device_type);
   
    // Define Subscriber
    // sync_.registerCallback(boost::bind(&SensorFusion::process, this, _1, _2));

    // Init counter for publishing
    time_frame_ = 0;
}

SensorFusion::~SensorFusion(){

}

void SensorFusion::process(const Image::ConstPtr & image){
	auto start = std::chrono::high_resolution_clock::now();
	processImage(image);
	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

	// Increment time frame
	time_frame_++;
}

void SensorFusion::processImage(const Image::ConstPtr & image){

/******************************************************************************
 * 1. Load precalculated semantic segmentated images to ensure online
 * performance
 */

	cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	cv::Mat raw_image = img_ptr -> image;
	// inference
	std::vector<std::vector<Detection>> result = yolo_detector_ptr_->Run(raw_image, params_.conf_thres, params_.iou_thres);
	// rect and text added
	viz_yolo_detections(raw_image, result);
 
  // get detection result of bbox, confidence, class_idx
  for (int i = 0; i < int(result.size()); ++i){
    for (int idx = 0; idx < int(result[i].size()); ++idx){
	cv::Rect rect = result[i][idx].bbox;
	float score = result[i][idx].score;
	int class_idx = result[i][idx].class_idx;

	int x, y, width, height;
	x = rect.x;
	y = rect.y;
	width = rect.width;
	height = rect.height;

	std::string class_name = class_names_[class_idx];

	int color_idx = SEMANTIC_NAME_TO_INDEX[class_names_[class_idx]];
	int sem_color_r = SEMANTIC_CLASS_TO_COLOR(color_idx,0);
	int sem_color_g = SEMANTIC_CLASS_TO_COLOR(color_idx,1);
	int sem_color_b = SEMANTIC_CLASS_TO_COLOR(color_idx,2);
      
	Object object;
	object.id = idx;
	object.x = x;
	object.y = y;
	object.width = width;
	object.height = height;
	object.semantic_name = class_name;
	object.semantic_confidence = score;
	object.semantic_id = class_idx;
	object.r = sem_color_r;
	object.g = sem_color_g;
	object.b = sem_color_b;
      
    object_array_.list.push_back(object);
    }
  }

	// Publish
  	object_array_pub_.publish(object_array_);
 
	cv_bridge::CvImage cv_yolo_result;
	cv_yolo_result.image = raw_image;
	cv_yolo_result.encoding = "bgr8";
	cv_yolo_result.header.stamp = image->header.stamp;
	yolo_detection_pub_.publish(cv_yolo_result.toImageMsg());
}


std::vector<std::string> SensorFusion::LoadNames(const std::string& path) {
	// load class names
	std::vector<std::string> class_names;
	std::ifstream infile(path);
	if (infile.is_open()) {
		std::string line;
		while (getline(infile,line)) {
			class_names.emplace_back(line);
		}
		infile.close();
	}
	else {
		std::cerr << "Error loading the class names!\n";
	}

	return class_names;
}


void SensorFusion::viz_yolo_detections(cv::Mat& img,
	const std::vector<std::vector<Detection>>& detections) {

	if (!detections.empty()) {
		for (const auto& detection : detections[0]) {
			const auto& box = detection.bbox;
			float score = detection.score;
			int class_idx = detection.class_idx;
			if(class_names_[class_idx] == "person" || class_names_[class_idx] == "car" || class_names_[class_idx] == "bicycle" 
				|| class_names_[class_idx] ==  "bus" || class_names_[class_idx] ==  "truck"){
				int color_idx = SEMANTIC_NAME_TO_INDEX[class_names_[class_idx]];
				int sem_color_r = SEMANTIC_CLASS_TO_COLOR(color_idx,0);
				int sem_color_g = SEMANTIC_CLASS_TO_COLOR(color_idx,1);
				int sem_color_b = SEMANTIC_CLASS_TO_COLOR(color_idx,2);

				cv::rectangle(img, box, cv::Scalar(sem_color_b, sem_color_g, sem_color_r), 2);

				if (true) {
					std::stringstream ss;
					ss << std::fixed << std::setprecision(2) << score;
					std::string s = class_names_[class_idx] + " " + ss.str();

					auto font_face = cv::FONT_HERSHEY_DUPLEX;
					auto font_scale = 1.0;
					int thickness = 1;
					int baseline=0;
					auto s_size = cv::getTextSize(s, font_face, font_scale, thickness, &baseline);
					cv::rectangle(img,
					            cv::Point(box.tl().x, box.tl().y - s_size.height - 5),
					            cv::Point(box.tl().x + s_size.width, box.tl().y),
					            // cv::Scalar(0, 0, 255), -1);
					            cv::Scalar(sem_color_b, sem_color_g, sem_color_r), -1);
					cv::putText(img, s, cv::Point(box.tl().x, box.tl().y - 5),
					          font_face , font_scale, cv::Scalar(255, 255, 255), thickness);
				}
			}
		}
	}
	// cv::namedWindow("Resmult", cv::WINDOW_AUTOSIZE);
	// cv::imshow("Result", img);
	// cv::waitKey(1);
}

void SensorFusion::InitParam() {
  // Fill transformation matrices

  SEMANTIC_NAMES = std::vector<std::string>{
      // Static objects
      "Road", "Sidewalk", "Building", "Wall", "Fence", "Pole",
      "Traffic light", "Traffic sign", "Vegetation", "Terrain", "Sky",
      // Dynamic objects
      "Pedestrian", "Rider", "Car", "truck", "bus", "Train", "Motocycle", "Bicycle"
  };

  SEMANTIC_NAME_TO_INDEX = std::map<std::string, int>{
      {"Road", 0}, {"Sidewalk", 1}, {"Building", 2}, {"Wall", 3}, {"Fence", 4}, {"Pole", 5},
      {"Traffic light", 6}, {"Traffic sign", 7}, {"Vegetation", 8}, {"Terrain", 9}, {"Sky", 10},

      {"person", 11}, {"Rider", 12}, {"car", 13}, {"truck", 14}, {"bus", 15}, {"Train", 16}, {"Motocycle", 17}, {"bicycle", 18}
  };

  SEMANTIC_COLOR_TO_CLASS = std::map<int, int>{
      // Static objects
      {320, 0}, {511, 1}, {210, 2}, {260, 3}, {496, 4}, {459, 5},
      {450, 6}, {440, 7}, {284, 8}, {555, 9}, {380, 10},
      // Dynamic objects
      {300, 11}, {255, 12}, {142, 13}, {70, 14},{160, 15}, {180, 16}, {230, 17}, {162, 18}
  };

  SEMANTIC_CLASS_TO_COLOR = MatrixXi::Zero(19, 3);
  SEMANTIC_CLASS_TO_COLOR <<
      // Static objects
      128,  64, 128, // Road
      244,  35, 232, // Sidewalk
      70,  70,  70, // Building
      102, 102, 156, // Wall
      190, 153, 153, // Fence
      153, 153, 153, // Pole
      250, 170,  30, // Traffic light
      220, 220,   0, // Traffic sign
      107, 142,  35, // Vegetation
      152, 251, 152, // Terrain
      70, 130, 180, // Sky

      // Dynamic objects
      220,  20,  60, // Pedestrian 11
      255,   0,   0, // Rider 12
      0,   0, 142, // Car 13
      0,   0,  70, // Truck 14
      0,  60, 100, // Bus 15
      0,  80, 100, // Train 16
      0,   0, 230, // Motocycle 17
      119,  11,  32;  // Bicycle 18

}

} // namespace sensor_processing
