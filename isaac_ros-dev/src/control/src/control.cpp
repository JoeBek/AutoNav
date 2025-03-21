

#include <rclcpp/rclcpp.hpp>
#include "serial.hpp"
#include "xbox.cpp"
#include "sensor_msgs/msg/joy.hpp"

class ControlNode : public rclcpp::Node {

    public:

    ControlNode() : Node("control_node"), serial("/dev/pts/15", 115200), xbox(){


        motor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControlNode::callback, this)
        );

        game_pass_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ControlNode::joystick_callback, this, std::placeholders::_1));
        
        
    }


    private:

    Serial serial;

    // motor controller subscription
    rclcpp::TimerBase::SharedPtr motor_timer_;

    // subscription for joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    Xbox::Xbox xbox;

    void motor_callback(){

        std::string message = "hello from autonav\n";

        serial.send(message);

        std::string buffer;

        serial.read_string(buffer);

        RCLCPP_INFO(this->get_logger(), "received string %s", buffer.c_str());

    }

    void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {

        bool b_button = joy_msg->buttons[1];    
        bool x_button = joy_msg->buttons[2];    
        bool y_button = joy_msg->buttons[3];  
        float left_stick_x = joy_msg->axes[0];    
        float left_stick_y = joy_msg->axes[1];    
        float right_stick_x = joy_msg->axes[2];    
        float right_stick_y = joy_msg->axes[3];    
        float left_trigger = joy_msg->axes[4];    
        float right_trigger = joy_msg->axes[5];

        // TODO add logic for sending to motors

        
        
    }



};


int main(int argc, char** argv) {

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;

}