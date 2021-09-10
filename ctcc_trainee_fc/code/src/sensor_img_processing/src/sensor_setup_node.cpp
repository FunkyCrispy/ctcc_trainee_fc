/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 19/04/2018
 *
 */

#include <ros/ros.h>
#include <sensor_processing_lib/sensor_fusion.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "sensor_setup_node");
	// namespaces are /node_namespace and /node_namespace/nodename
	sensor_img_processing::SensorFusion sensor_setup(
		ros::NodeHandle(), ros::NodeHandle("~"));
	ros::spin();

	return 0;
}
