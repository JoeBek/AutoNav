#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// node declaration
class Planner : public rclcpp::Node
{
public:

  // Node constructor
  Planner() : Node("Planner")
  {

    // publisher object constructor.
     publisher_ = this->create_publisher<sensor_msgs::msg::Image>("Object_List", 10);

    // Subscriber object constrcutor. The call to bind will attach a callback function to the class instance
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/zed/zed_node/left/image_rect_color", 10,
      std::bind(&Planner::callback, this, std::placeholders::_1));
  }

private:


  void callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
  }

  // declarations of the publishers and subscribers. The type is the only thing you'll adjust.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planner>());
  rclcpp::shutdown();
  return 0;
}


