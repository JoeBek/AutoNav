#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstring>
#include <cstdlib>

#include "serialib.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "std_msgs/msg/string.hpp"
#include "autonav_interfaces/msg/encoders.hpp"

using namespace std::chrono_literals;

class WheelOdomPublisher : public rclcpp::Node
{
  public:
    WheelOdomPublisher()
    : Node("wheelodom_publisher"), x_(0.0), y_(0.0), theta_(0.0), linear_velocity_(0.0), angular_velocity_(0.0), 
    wheel_base_(0.6858), wheel_radius_(0.205), prev_left_encoder_count_(0), prev_right_encoder_count_(0), 
    left_encoder_(0), right_encoder_(0), ticks_per_revolution_(81923)
    {
      encoder_subscription_ = this->create_subscription<autonav_interfaces::msg::Encoders>("encoders", 
      10, std::bind(&WheelOdomPublisher::encoder_callback, this, std::placeholders::_1));

      publisher_ = this->create_publisher<std_msgs::msg::String>("wheel_odom", 50); //CHAGE PUBLISHER TYPE TO NAV ODOM
      timer_ = this->create_wall_timer(200ms, std::bind(&WheelOdomPublisher::update_wheel_odom, this));
      last_time_ = this->get_clock()->now();
    }

    // void simulateEncoderCounts(int left, int right)
    // {
    //     left_encoder_count_ = left;
    //     right_encoder_count_ = right;
    // }

  private:
    void encoder_callback(const autonav_interfaces::msg::Encoders::SharedPtr msg)
    {
      // Update encoder values when new data is received
      left_encoder_ = msg->left_motor_count;
      right_encoder_ = msg->right_motor_count;
    }

    void update_wheel_odom()
    {
      // Get the current time and calculate the time difference (dt)
      auto current_time = this->get_clock()->now();
      double dt = (current_time - last_time_).seconds();  // Time in seconds
      last_time_ = current_time;  // Update last_time_ for the next callback

      // Calculate the change in encoder counts
      int left_delta_ticks = left_encoder_count_ - prev_left_encoder_count_;
      int right_delta_ticks = right_encoder_count_ - prev_right_encoder_count_;

      // Update previous encoder counts for next iteration
      prev_left_encoder_count_ = left_encoder_count_;
      prev_right_encoder_count_ = right_encoder_count_;

      // Convert encoder ticks to linear displacement (in meters)
      double left_displacement = (2 * M_PI * wheel_radius_) * (left_delta_ticks / (double)ticks_per_revolution_);
      double right_displacement = (2 * M_PI * wheel_radius_) * (right_delta_ticks / (double)ticks_per_revolution_);
      
      // Compute robot's linear velocity and angular velocity
      double forward_displacement = (left_displacement + right_displacement) / 2.0;
      double angular_displacement = (right_displacement - left_displacement) / wheel_base_;
      
      // Update robot's position (x, y) and orientation (theta)
      x_ += forward_displacement * cos(theta_);
      y_ += forward_displacement * sin(theta_);
      theta_ += angular_displacement;

      // Normalize theta to be within [-pi, pi]
      if (theta_ > M_PI)
          theta_ -= 2 * M_PI;
      if (theta_ < -M_PI)
          theta_ += 2 * M_PI;

      // Calculate linear and angular velocities
      linear_velocity_ = forward_displacement / dt;  // meters per second (m/s)
      angular_velocity_ = angular_displacement / dt;  // radians per second (rad/s)

      // Print x, y, theta, linear, and angular velocity for debugging
      printf("Position: x = %f, y = %f, theta = %f\n", x_, y_, theta_);
      printf("Velocity: linear = %f, angular = %f\n", linear_velocity_, angular_velocity_);
    }

    double x_, y_, theta_;  // Robot's position (x, y) and orientation (theta)
    double linear_velocity_, angular_velocity_; // Robot's linear and angular velocity 
    double wheel_base_;  // Distance between wheels (L)
    double wheel_radius_;  // Radius of the wheels (r)
    int left_encoder_count_, right_encoder_count_;  // Current encoder counts
    int prev_left_encoder_count_, prev_right_encoder_count_;  // Previous encoder counts for delta calculation
    int32_t left_encoder_; // Left Encoder count reading from control topic
    int32_t right_encoder_; // Right Encoder count reading from control topic
    const int ticks_per_revolution_;  // Number of encoder ticks per wheel revolution
    rclcpp::Time last_time_; //Time of the last callback to calculate dt

    serialib motorController;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<autonav_interfaces::msg::Encoders>::SharedPtr encoder_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdomPublisher>());
  //auto odometry_node = std::make_shared<WheelOdomPublisher>();
  //odometry_node->simulateEncoderCounts(80000, 80000);  // First update (left = 1000, right = 1200)
  //std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait 1 second for odometry update
  //odometry_node->simulateEncoderCounts(40000, 120000);  // Second update (left = 2000, right = 2400)
  //std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait 1 second for odometry update
  //rclcpp::spin(odometry_node);
  
  rclcpp::shutdown();
  return 0;
}