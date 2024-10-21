// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    // presumably initializes the node as a publisher to topic 'topic' 
    publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);
    // instantiates timer to go off twice a second using callback function "callback".
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // auto is used because ros will decide your types
    auto message = tutorial_interfaces::msg::Num();
    message.num = this->count_++;
    // this is a macro that logs the message to the console
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'");
    // publishes the message
    publisher_->publish(message);
  }
  // timer variable points to object instance bound to timer_callback function.
  // so every time the timer hits 500 ms, it calls timer_callback.
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  // ROS setup
  rclcpp::init(argc, argv);
  // start node
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  // clean up. Easy as that.
  rclcpp::shutdown();
  return 0;
}
