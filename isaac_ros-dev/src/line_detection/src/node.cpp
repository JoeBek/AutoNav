#include "line_detection/detection.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "autonav_interfaces/srv/anv_lines.hpp"

#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <mutex>

#define DEBUG_LOG

class LineDetectorNode : public rclcpp::Node {

	public:

	LineDetectorNode() : Node("lines"),
	 tf_buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
	  tf_listener(tf_buffer) {

		// set parameters

		this->declare_parameter("camera_topic", "rgb_gray/image_rect_gray");

		std::string camera_topic = this->get_parameter("camera_topic").as_string();

		// subscribe to zed topic
		   auto get_latest_msg = [this](sensor_msgs::msg::Image::SharedPtr msg) {
				std::lock_guard<std::mutex> lock(callback_lock);
				latest_img = msg;
		   };
		   _zed_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
		   camera_topic, 10, get_latest_msg);

			// create service for line detection
		   _line_service = this->create_service<autonav_interfaces::srv::AnvLines>("line_service",
			 std::bind(&LineDetectorNode::line_service, this, std::placeholders::_1, std::placeholders::_2));
	   


	}


	private:

	// TODO create publisher for cost map
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _zed_subscriber;

	rclcpp::Service<autonav_interfaces::srv::AnvLines>::SharedPtr _line_service;

	std::mutex callback_lock;
	sensor_msgs::msg::Image::SharedPtr latest_img;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;



	void line_service(const std::shared_ptr<autonav_interfaces::srv::AnvLines::Request> request,
					std::shared_ptr<autonav_interfaces::srv::AnvLines::Response> response);

	std::vector<Eigen::Vector3d> map_transform(const sensor_msgs::msg::Image::SharedPtr depth_msg, int2* line_points, int line_points_len); 



};

/**
 * Converts a list of image indicies to depth image indicies
 */
std::vector<Eigen::Vector3d> LineDetectorNode::map_transform(const sensor_msgs::msg::Image::SharedPtr depth_msg, int2* line_points, int line_points_len) {

    // validate depth image format
    if (depth_msg->encoding != "16UC1") {
      RCLCPP_WARN(get_logger(), "Unsupported depth format: %s", 
                 depth_msg->encoding.c_str());
      return;
    }

    // extract depth value (in millimeters) TODO validate
    const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(depth_msg->data.data());

	std::vector<Eigen::Vector3d> depth_line_points;

	for (int i=0; i < line_points_len; i++){

		const uint16_t depth_mm = depth_data[line_points[i].y * depth_msg->width + line_points[i].x];
		
		if (depth_mm == 0) {
		RCLCPP_WARN(get_logger(), "Invalid depth at (%d,%d)", target_u, target_v);
		return;
		}

		// Convert to meters and project to 3D
		const double depth_m = depth_mm / 1000.0;
		const cv::Point3d ray = camera_model_.projectPixelTo3dRay(
		cv::Point2d(target_u, target_v));
		
		// Create camera-frame point
		geometry_msgs::msg::PointStamped camera_point;
		camera_point.header = depth_msg->header;
		camera_point.point.x = ray.x * depth_m;
		camera_point.point.y = ray.y * depth_m;
		camera_point.point.z = ray.z * depth_m;

		// Transform to map frame
		try {
		geometry_msgs::msg::PointStamped map_point = 
			tf_buffer_.transform(camera_point, "map", tf2::durationFromSec(1.0));
		
		RCLCPP_INFO(this->get_logger(), "Map coordinates: (%.2f, %.2f, %.2f)",
					map_point.point.x, map_point.point.y, map_point.point.z);
		
		// append to map point coords
		depth_line_points.emplace_back(camera_point.x, camera_points.y, 0);
		
		} catch (const tf2::TransformException& ex) {
		RCLCPP_ERROR(get_logger(), "TF error: %s", ex.what());

		}


	}

	return depth_line_points;
}

void LineDetectorNode::line_service(const std::shared_ptr<autonav_interfaces::srv::AnvLines::Request> request,
					std::shared_ptr<autonav_interfaces::srv::AnvLines::Response> response)
{

	// take camera data, turn it into cv::Mat 8UC1, and simply call detect
	// detect will return a pointer to an array of (x,y) line points,
	// and the length of the array. 

	(void)request;

	// read latest camera message thread safe
	sensor_msgs::msg::Image::SharedPtr camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(callback_lock);
		return latest_img;
		
	}();

	#ifdef DEBUG_LOG
	RCLCPP_INFO(this->get_logger(), "camera message read");
	#endif

	// convert camera image to cv::Mat for line detection
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::MONO8);  
	} catch (cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat camera_img = cv_ptr->image;

	#ifdef DEBUG_LOG
	RCLCPP_INFO(this->get_logger(), "camera bridge complete");
	#endif

	// detect lines
	std::pair<int2*,int*> line_pair = lines::detect_line_pixels(camera_img);
	int2* line_points;
	int* line_points_len;
	std::tie(line_points, line_points_len) = line_pair;

	#ifdef DEBUG_LOG
	RCLCPP_INFO(this->get_logger(), "lines detected. points: %d", *line_points_len);
	#endif
	

	#ifdef TEMP
	// get map -> camera link transform
	geometry_msgs::msg::TransformStamped transform;
	try {
		// TODO get zed link actual name
		transform = tf_buffer.lookupTransform("map", "camera_link", tf2::TimePointZero);
	}
	catch (tf2::TransformException &e) {
		RCLCPP_ERROR(this->get_logger(), "%s", e.what());
		return;
	}

	#ifdef DEBUG_LOG
	RCLCPP_INFO(this->get_logger(), "transform complete");
	#endif
	
	
	// convert transform, line points to eigen vectors (oooh I get why they call it that)
	Eigen::Affine3d tf_matrix = tf2::transformToEigen(transform);
	std::vector<Eigen::Vector3d> camera_points;
	for (int i = 0; i < *line_points_len; i++) {

		camera_points.emplace_back(line_points[i].x, line_points[i].y, 0);
		
	}
	std::vector<Eigen::Vector3d> transform_points_camera;
	transform_points_camera.reserve(camera_points.size());

	// apply transform to get map points
	std::transform(camera_points.begin(), camera_points.end(), std::back_inserter(transform_points_camera),
					[&tf_matrix](const auto &point) {
							return tf_matrix * point;
	});

	#endif


	std::vector<Eigen::Vector3d> map_points = map_transform(/*topic*/, line_points, *line_points_len);


	#ifdef DEBUG_LOG
	RCLCPP_INFO(this->get_logger(), "transform 2 complete");
	#endif
	


	// populate service response
	for (const auto & point: map_points) {
		geometry_msgs::msg::Vector3 vec_msg;
		vec_msg.x = point.x();
		vec_msg.y = point.y();
		vec_msg.z = point.z();
		response->points.emplace_back(vec_msg);
	}


	// free mem
	delete[] line_points;
	delete line_points_len;

}



int main(int argc, char** argv) {

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<LineDetectorNode>());
	rclcpp::shutdown();
	return 0;
}

	
	
