#include <rclcpp/rclcpp.hpp>
#include "serialib.hpp"
#include "xbox.hpp"
#include "motor_controller.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include <queue>
#include <iostream>

//#include "autonav_interfaces/msg/Encoders.msg"

class ControlNode : public rclcpp::Node {

    public:

    ControlNode() 
      : Node("control_node")
       {
        
        /*encoder_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControlNode::publish_encoder_data, this)
        );*/

        initialize_serial_connections();

        //XBOX SUB
        controllerSub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ControlNode::joystick_callback, this, std::placeholders::_1));
        
        //PATH PLANNING SUB
        /*pathPlanningSub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ControlNode::path_planning_callback, this, std::placeholders::_1));
        
        
        //NAVIGATION ENCODER PUB
        //navigationEncoderPub = this->create_publisher<autonav_interfaces::msg::Encoders>("encoder_data", 10);


        //GPS PUB
        gpsPub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ControlNode::joystick_callback, this, std::placeholders::_1));

        arduinoSerial.write("0");*/
    }


    private:

    serialib arduinoSerial;
    serialib gpsSerial;
    Xbox controller;
    MotorController motors;

    bool autonomousMode = false;

    //rclcpp::TimerBase::SharedPtr encoder_timer_;

    // subscription for joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controllerSub;

    //rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pathPlanningSub;

    

    void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        std::cout << "Test";
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

            Xbox::CommandData command = controller.calculateCommand();

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
                motors.shutdown();
            }
            else if (command.cmd == Xbox::CHANGE_MODE){
                autonomousMode = 1;
                char buffer[3] = "1\n";
                arduinoSerial.writeString(buffer);
            }
        }
    }



    // void path_planning_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    //     struct AutonomousCmd{
    //         double linearX;
    //         double linearY;
    //         double angularZ;
    //     }
    //     std::queue<AutonomousCmd> pathCommands;

    //     if (autonomousMode) {
    //         for(int i = 0; i < msg.size(); i++){
    //             AutonomousCmd cmd;
    //             cmd.linearX = msg[i].linearX;
    //             cmd.linearY = msg[i].linearY;
    //             cmd.angularZ = msg[i].angularZ;
    //             pathCommands.push(cmd);
    //         }

    //         while(!queue.empty()){
                    

    //             queue.pop();
    //         }
    //     }
    // }

    /*void publish_encoder_data() {
        autonav_interfaces::msg::Encoders encoder_msg;
        encoder_msg.leftMotorRPM = motors.getLeftMotorRPM();
        encoder_msg.rightMotorRPM = motors.getRightMotorRPM();
        navigationEncoderPub->publish(encoder_msg);

        arduinoSerial.write("L" + std::to_string(encoder_msg.leftMotorRPM) + " R" + std::to_string(encoder_msg.rightMotorRPM));
    }*/

    void initialize_serial_connections() {
        // Open all serial connections on startup
        //arduinoSerial.openDevice("/dev/ttyACM#", 115200);
        //gpsSerial.openDevice("/dev/ttyACM#", 115200);                        
    }

};


int main(int argc, char** argv) {

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;

}