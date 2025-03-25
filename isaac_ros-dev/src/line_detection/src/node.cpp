#include <rclcpp/rclcpp.hpp>
#include "detection.hpp"
#include <sensor_msgs/Image>
#include "autonav_interfaces/msg/anv_point_list.hpp"
#include "autonav_interfaces/srv/anv_lines.hpp"

#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv2.hpp>
#include <buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <mutex>


class LineDetectorNode : public rclcpp:Node {

	public:

	LineDetectorNode : Node("Line Detector Node") {

		// subscribe to zed topic
		   auto latest_msg = [this](sensor_msgs::Image::SharedPtr msg) {
				std::lock_guard<std::mutex> lock(callback_lock);
				latest_msg = msg;
		   };
		   zed_img = this->create_subscription<sensor_msgs::Image>(
		   "Image", 10, latest_msg);

		   line_service = this->create_service<autonav_interfaces::srv::anv_lines>("line_service",
			 std::bind(&LineDetectorNode::line_service, this, std::placeholders::_1, std::placeholders::_2));

		   
		   // TODO create publisher for cost map

		   tf_buffer = tf2_ros::Buffer(this->get_clock());
		   tf_listener = tf2_ros::TransformListener(tf_buffer);
		   

	}


	private:

	// TODO create publisher for cost map
	rclcpp::Subscription<sensor_msgs::Image>::SharedPtr camera_sub_;

	rclcpp::Service<autonav_interfaces::srv::anv_lines>::SharedPtr line_service;

	std::mutex callback_lock;
	sensor_msgs::Image latest_img;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;


	void LineDetectorNode::line_service(const std::shared_ptr<autonav_interfaces::srv::anv_lines::Request,
					std::shared_ptr<autonav_interfaces::srv::anv_lines::Response);


}

void LineDetectorNode::line_service(const std::shared_ptr<autonav_interfaces::srv::anv_lines::Request,
					std::shared_ptr<autonav_interfaces::srv::anv_lines::Response)
{

	// take camera data, turn it into cv::Mat 8UC1, and simply call detect
	// detect will return a pointer to an array of (x,y) line points,
	// and the length of the array. 

	(void)request;

	// read latest camera message thread safe
	sensor_imgs::Image camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(callback_lock);
		return latest_img;
		
	}();

	// convert camera image to cv::Mat for line detection
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::MONO8);  
	} catch (cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat camera_img = cv_ptr->image;

	// detect lines
	std::pair<int2*,int*> line_pair = lines::detect_line_pixels(camera_img);
	int2* line_points;
	int* line_points_len;
	std::tie(line_points, line_points_len) = line_pair;

	// get map -> camera link transform
	geometry_msgs::msg::TransformStamped transform;
	try {
		// TODO get zed link actual name
		transform = tf_buffer.lookupTransform("map", "zed_link", tf2::TimePointZero);
	}
	catch (tf2::TransformException &e) {
		RCLCPP_ERROR(this->get_logger(), %s, ex.what());
		return;
	}
	
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


	// populate service response
	for (const auto & point: transform_points_camera) {
		Geometry_msgs::msg::Vector3 vec_msg;
		vec_msg.x = point.x();
		vec_msg.y = point.y();
		vec_msg.z = point.z();
		response->anv_points.push_back(vec_msg);
	}

	


}




int main(int argc, char** argv) {

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<LineDetectorNode>());
	rclcpp::shutdown();
	return 0;
}

	
	
