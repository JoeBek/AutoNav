#include <rclcpp/rclcpp.hpp>
#include "serial.hpp"
#include "xbox.hpp"
#include "sensor_msgs/msg/joy.hpp"

class ControlNode : public rclcpp::Node {

    public:

    ControlNode() : Node("control_node"), motorSerial("/dev/pts/15", 115200), arduinoSerial("idk yet,", 9600), xbox(), MotorController("idk yet"){


        /*motor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControlNode::callback, this)
        );*/

        initialize_serial_connections();

        game_pass_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ControlNode::joystick_callback, this, std::placeholders::_1));
        
        
    }


    private:

    Serial motorSerial;
    Serial arduinoSerial;
    Xbox controller;
    MotorController motors;

    bool autonomousMode = false;

    // motor controller subscription
    rclcpp::TimerBase::SharedPtr motor_timer_;

    // subscription for joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    Xbox::Xbox xbox;

    //read test
    void motor_callback(){

        std::string message = "hello from autonav\n";

        serial.send(message);

        std::string buffer;

        serial.read_string(buffer);

        RCLCPP_INFO(this->get_logger(), "received string %s", buffer.c_str());

    }

    void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {

        if(!autonomousMode){
            controller.set_b(joy_msg->buttons[1]);
            controller.set_x(joy_msg->buttons[2]);
            controller.set_y(joy_msg->buttons[3]);

            controller.set_right_bumper(joy_msg->buttons[4]);
            controller.set_left_bumper(joy_msg->buttons[5]);

            controller.set_left_stick_x(joy_msg->axes[0]);
            controller.set_left_stick_y(joy_msg->axes[1]);
            controller.set_right_stick_x(joy_msg->axes[2]);
            controller.set_right_stick_y(joy_msg->axes[3]);

            CommandData command = controller.calculateCommand();

            if(command.cmd == Xbox::MOVE){
                motors.move(command.right_motor_speed * motors.getSpeed(), command.left_motor_speed * motors.getSpeed());
            }
            else if(command.cmd == Xbox::SPEED_DOWN){
                motors.setSpeed(motors.getSpeed() - 10);
            }
            else if(command.cmd == Xbox::SPEED_UP){
                motors.setSpeed(motors.getSpeed() + 10);
            }
            else if(command.cmd == Xbox::STOP){
                motors.shutDown();
            }
            else if (command.cmd == Xbox::CHANGE_MODE){
                autonomousMode = true;
            }
        }
    }

    void initialize_serial_connections() {
        // Open all serial connections on startup
        motorSerial.open();
        arduinoSerial.open();            
    }

};


int main(int argc, char** argv) {

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;

}