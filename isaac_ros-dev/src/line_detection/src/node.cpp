#include <rclcpp/rclcpp.hpp>
#include "detection.hpp"
#include <sensor_msgs/Image>
#include "autonav_interfaces/msg/anv_point_list.hpp"
#include "autonav_interfaces/srv/anv_lines.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv2.hpp>

#include <mutex>



class LineDetectorNode : public rclcpp:Node {

	public:

	LineDetectorNode : Node("Line Detector Node") {

				   // subscribe to zed topic
		
		   auto latest_msg = [this](sensor_msgs::Image::SharedPtr msg) {
				std::lock_guard<std::mutex> lock(callback_lock);
				latest_msg = msg;
		   }
		   zed_img = this->create_subscription<sensor_msgs::Image>(
		   "Image", 10, latest_msg);

		   line_service = this->create_service<autonav_interfaces::srv::anv_lines>("line_service", &line_service);

		   
		   // TODO create publisher for cost map

	}


	private:

	// TODO create publisher for cost map
	rclcpp::Subscription<sensor_msgs::Image>::SharedPtr camera_sub_;

	rclcpp::Service<autonav_interfaces::srv::anv_lines>::SharedPtr line_service;

	std::mutex callback_lock;
	sensor_msgs::Image latest_img;


}

void line_service(const std::shared_ptr<autonav_interfaces::srv::anv_lines::Request,
					std::shared_ptr<autonav_interfaces::srv::anv_lines::Response)
{

	// take camera data, turn it into cv::Mat 8UC1, and simply call detect
	// detect will return a pointer to an array of (x,y) line points,
	// and the length of the array. They will arrive as device pointers,
	// which can be directly mapped to the cost map. 

	(void)request;

	// read latest camera message thread safe
	sensor_imgs::Image camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(callback_lock);
		return latest_img;
		
	}();

	// convert camera image to cv::Mat
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(camera_image, sensor_msgs::image_encodings::MONO8);  
	} catch (cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat camera_img = cv_ptr->image;

	std::pair<int2*,int*> line_pair = lines::detect_line_pixels(camera_img);


}




int main(int argc, char** argv) {

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<LineDetectorNode>());
	rclcpp::shutdown();
	return 0;
}

	
	
