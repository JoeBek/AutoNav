#include <rclcpp/rclcpp.hpp>
#include "detection.hpp"
#include <sensor_msgs/Image>




class LineDetectorNode : public rclcpp:Node {

	public:

	LineDetectorNode : Node("Line Detector Node") {

				   // subscribe to zed topic
		
		   zed_img = this->create_subscription<sensor_msgs::Image>(
		   "Image", 10, std::bind(&LineDetectorNode::callback, this, std::placeholders::_1)); // TODO


		   // TODO create publisher for cost map

	}


	private:

	// TODO create publisher for cost map
	rclcpp::Subscription<sensor_msgs::Image>::SharedPtr camera_sub_;


}

void camera_callback(){

	// take camera data, turn it into cv::Mat 8UC1, and simply call detect
	// detect will return a pointer to an array of (x,y) line points,
	// and the length of the array. They will arrive as device pointers,
	// which can be directly mapped to the cost map. 

}


int main(int argc, char** argv) {

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<LineDetectorNode>());
	rclcpp::shutdown();
	return 0;
}

	
	
