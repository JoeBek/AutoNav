#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// node declaration
class ZedImageSubscriber : public rclcpp::Node
{
public:

  // constructor
  ZedImageSubscriber() : Node("zed_image_subscriber")
  {

     publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);

    // create topic subscription
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/zed/zed_node/left/image_rect_color", 10,
      std::bind(&ZedImageSubscriber::image_callback, this, std::placeholders::_1));
  }

private:
  // image 
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    try
    {
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::Mat edges;
      cv::Canny(image, edges, 100, 200);
      publishProcessedImage(edges);
      cv::imshow("ZED Left Image", edges);
      cv::waitKey(1);


    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
    }
  }

    void publishProcessedImage(const cv::Mat& processed_image) const
  {
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_frame"; // Set appropriate frame_id

    sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(header, "bgr8", processed_image).toImageMsg();
    publisher_->publish(*output_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}

