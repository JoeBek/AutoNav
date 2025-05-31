#include "line_detection/detection.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "autonav_interfaces/srv/anv_lines.hpp"
#include "autonav_interfaces/msg/line_points.hpp"

#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>  // For tf2::getTimestamp, tf2::getFrameI
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <image_geometry/pinhole_camera_model.h>
#include <mutex>

//#define DEBUG_2
//#define DEBUG_3

//#define DEBUG_LOG

class LineDetectorNode : public rclcpp::Node {

	public:

	LineDetectorNode() : Node("lines"),
	 tf_buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
	  tf_listener(tf_buffer) {

		// set parameters

		this->declare_parameter("camera_topic", "rgb_gray/image_rect_gray");
		// TODO TODO OOODO fix this and below 
		this->declare_parameter("depth_camera_topic", "rgb_gray/depth/raw");
		this->declare_parameter("camera_info_topic", "rgb_gray/info");
		this->declare_parameter("line_points_topic", "line_points");
		this->declare_parameter("enable_timer", true); 
		

		std::string camera_topic = this->get_parameter("camera_topic").as_string();
		std::string depth_camera_topic = this->get_parameter("depth_camera_topic").as_string();
		std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
		std::string line_points_topic = this->get_parameter("line_points_topic").as_string();
		this->get_parameter("enable_timer", enable_timer_);

		// subscribe to zed topic
		auto get_latest_msg = [this](sensor_msgs::msg::Image::SharedPtr msg) {
				std::lock_guard<std::mutex> lock(callback_lock);
				latest_img = msg;
		   };
		auto get_latest_depth_msg = [this](sensor_msgs::msg::Image::SharedPtr msg) {
				std::lock_guard<std::mutex> lock(depth_callback_lock);
				latest_depth_img = msg;
		   };
		_zed_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
		   camera_topic, 10, get_latest_msg);

		_zed_depth_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
		   depth_camera_topic, 10, get_latest_depth_msg);

		_camera_model_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
			camera_info_topic, 1, std::bind(&LineDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

			// PUBLISHERS 
		_line_pub = this->create_publisher<autonav_interfaces::msg::LinePoints>(
				line_points_topic, 1
			);
		_line_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LineDetectorNode::line_callback, this));

		_line_point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("lines_pointcloud", 10)
			// create service for line detection
		    _line_service = this->create_service<autonav_interfaces::srv::AnvLines>("line_service",
			 std::bind(&LineDetectorNode::line_service, this, std::placeholders::_1, std::placeholders::_2));
			
			

	}


	private:

	// TODO create publisher for cost map
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _zed_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _zed_depth_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_model_sub;

	rclcpp::Service<autonav_interfaces::srv::AnvLines>::SharedPtr _line_service;

	std::mutex callback_lock;
	std::mutex depth_callback_lock;
	sensor_msgs::msg::Image::SharedPtr latest_img;
	sensor_msgs::msg::Image::SharedPtr latest_depth_img;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	image_geometry::PinholeCameraModel camera_model_;
	bool enable_timer_;
	bool configured_ = false;

	// publisher for lines
	rclcpp::Publisher<autonav_interfaces::msg::LinePoints>::SharedPtr _line_pub;
	rclcpp::TimerBase::SharedPtr _line_timer;

	void line_service(const std::shared_ptr<autonav_interfaces::srv::AnvLines::Request> request,
					std::shared_ptr<autonav_interfaces::srv::AnvLines::Response> response);
	
	void line_callback();

	std::vector<Eigen::Vector3d> map_transform(const sensor_msgs::msg::Image::SharedPtr depth_msg, int2* line_points, int line_points_len); 

	void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);


};

// gets camera params
void LineDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!camera_model_.initialized()) {
        camera_model_.fromCameraInfo(*msg);
		RCLCPP_INFO(this->get_logger(), "hello, its me, bowser. I am sentient. tell no one");
		configured_ = true;
		if (enable_timer_) {
			RCLCPP_INFO(this->get_logger(), 
			"publishing enabled. tune in at %s", this->get_parameter("line_points_topic").as_string().c_str());
		}
		
    }
}


/**
 * Converts a list of image indicies to map frame coordinates  
 * */
std::vector<Eigen::Vector3d> LineDetectorNode::map_transform(const sensor_msgs::msg::Image::SharedPtr depth_msg, int2* line_points, int line_points_len) {

	std::vector<Eigen::Vector3d> depth_line_points;

    // extract depth value TODO validate
    const float* depth_data = reinterpret_cast<const float*>(depth_msg->data.data());

	std::vector<std::array<float, 3>> pc_vec;


	for (int i=0; i < line_points_len; i++){

		const float depth_cm = depth_data[line_points[i].y * depth_msg->width + line_points[i].x];
		
		#ifdef DEBUG_2
		RCLCPP_INFO(get_logger(), "x: %d, y: %d, depth: %f\n", line_points[i].y, line_points[i].x, depth_cm);
		#endif
		if (depth_cm <= 0.0 || std::isnan(depth_cm)) {
			RCLCPP_WARN(get_logger(), "Invalid depth at (%d,%d)", line_points[i].x, line_points[i].y);
			continue;
		}

		#ifdef DEBUG_LOG
		RCLCPP_INFO(this->get_logger(), "detected image index: %d, %d", line_points[i].x, line_points[i].y);
		#endif

		// convert to meters and project to 3D
		const double depth_m = depth_cm / 100.0;
		const cv::Point3d ray = camera_model_.projectPixelTo3dRay(
		cv::Point2d(line_points[i].x, line_points[i].y));
		
		// create camera-frame point
		geometry_msgs::msg::PointStamped camera_point;
		camera_point.header = depth_msg->header;
		float point_x = ray.x * depth_cm;
		float point_y = ray.y * depth_cm;
		float point_z = ray.z * depth_cm;
		camera_point.point.x = point_x;
		camera_point.point.y = point_y;
		camera_point.point.z = point_z;

		pc_vec.push_back({point_x, point_y, point_z});
		RCLCPP_INFO(this->get_logger(), "points: %f, %f, %f", point_x, point_y, point_z);



		

		// transform to map frame
		try {

		geometry_msgs::msg::TransformStamped transform = tf_buffer.lookupTransform("map", camera_point.header.frame_id, tf2::TimePointZero);
		geometry_msgs::msg::PointStamped map_point;
		tf2::doTransform(camera_point,map_point, transform);
		
		#ifdef DEBUG_3
		RCLCPP_INFO(this->get_logger(), "Map coordinates: (%.2f, %.2f, %.2f)",
					map_point.point.x, map_point.point.y, map_point.point.z);
		#endif
		
		// append to map point coords
		depth_line_points.emplace_back(map_point.point.x, map_point.point.y, 0);
		
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
	sensor_msgs::msg::Image::SharedPtr depth_camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(depth_callback_lock);
		return latest_depth_img;
		
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
	

	std::vector<Eigen::Vector3d> map_points = map_transform(depth_camera_msg, line_points, *line_points_len);


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

void LineDetectorNode::line_callback()
{

	// take camera data, turn it into cv::Mat 8UC1, and simply call detect
	// detect will return a pointer to an array of (x,y) line points,
	// and the length of the array. 

	if (!configured_ || !enable_timer_){
		return;
	}

	// read latest camera message thread safe
	sensor_msgs::msg::Image::SharedPtr camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(callback_lock);
		return latest_img;
		
	}();
	sensor_msgs::msg::Image::SharedPtr depth_camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(depth_callback_lock);
		return latest_depth_img;
		
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
	

	std::vector<Eigen::Vector3d> map_points = map_transform(depth_camera_msg, line_points, *line_points_len);


	#ifdef DEBUG_LOG
	RCLCPP_INFO(this->get_logger(), "transform 2 complete");
	#endif
	

	auto message = autonav_interfaces::msg::LinePoints();
	// populate service response
	for (const auto & point: map_points) {
		geometry_msgs::msg::Vector3 vec_msg;
		vec_msg.x = point.x();
		vec_msg.y = point.y();
		vec_msg.z = point.z();
		message.points.emplace_back(vec_msg);

	}

	_line_pub->publish(message);


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

	
	
