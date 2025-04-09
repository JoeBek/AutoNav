
#include <rclcpp/rclcpp.hpp>
#include "serialib.hpp"
#include "xbox.hpp"
#include "motor_controller.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "autonav_interfaces/msg/encoders.hpp"
#include "autonav_interfaces/msg/gps_data.hpp"  
#include <queue>
#include <iostream>


class ControlNode : public rclcpp::Node {

    public:


    ControlNode() 
      : Node("control_node")
       {

        initialize_serial_connections();

        //XBOX SUB
        controllerSub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ControlNode::joystick_callback, this, std::placeholders::_1));

        //NAVIGATION ENCODER PUB
        navigationEncoderPub = this->create_publisher<autonav_interfaces::msg::Encoders>("encoder_topic", 10);
        
        encoder_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControlNode::publish_encoder_data, this)
        );

        //GPS PUB
        gpsPub = this->create_publisher<autonav_interfaces::msg::GpsData>("gps_topic", 10);

        gps_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControlNode::publish_gps_data, this)
        );
        
        //PATH PLANNING SUB
        /*pathPlanningSub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ControlNode::path_planning_callback, this, std::placeholders::_1));
        
        //GPS PUB
        gpsPub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ControlNode::joystick_callback, this, std::placeholders::_1));

        */
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
                autonomousMode = true;
                char mode[12] = "AUTONOMOUS\n";
                arduinoSerial.writeString(mode);
            }  
        }
        else{
            //TODO: logic for checking if B button is pressed 
            
        }
        
    }

    void publish_encoder_data() {
        autonav_interfaces::msg::Encoders encoder_msg;
        encoder_msg.left_motor_rpm = std::to_string(motors.getLeftRPM() / 20);
        encoder_msg.right_motor_rpm = std::to_string(motors.getRightRPM() / 20);

        std::string arduinoRPMs = "L:";
        arduinoRPMs += encoder_msg.left_motor_rpm;
        arduinoRPMs += " R:";
        arduinoRPMs += encoder_msg.right_motor_rpm;
        arduinoRPMs += "\n";
        arduinoSerial.writeString(arduinoRPMs.c_str());

        navigationEncoderPub->publish(encoder_msg);
    }

    void publish_gps_data() {
        autonav_interfaces::msg::GpsData gps_msg;
        char gpsBuffer[1024] = {};
        gpsSerial.readString(gpsBuffer, '\n', 1023, 1000);
        //gps_msg.latitude = gpsSerial.getLatitude();
        //gps_msg.longitude = gpsSerial.getLongitude();
        gpsPub->publish(gps_msg);
    }

    rclcpp::Publisher<autonav_interfaces::msg::GpsData>::SharedPtr gpsPub;
    rclcpp::TimerBase::SharedPtr gps_timer_;

    rclcpp::Publisher<autonav_interfaces::msg::Encoders>::SharedPtr navigationEncoderPub;
    rclcpp::TimerBase::SharedPtr encoder_timer_;


    // void path_planning_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    //     //TODO: send motor commands based on pose
    // }


    void initialize_serial_connections() {
        // Open all serial connections on startup
        arduinoSerial.openDevice("/dev/ttyTHS1", 9600);
        char mode[8] = "MANUAL\n";
        arduinoSerial.writeString(mode);

        gpsSerial.openDevice("/dev/ttyACM#", 115200);
        char gpsStartCmd[32] = "log bestpos ontime 2\r\n";
        gpsSerial.writeString(gpsStartCmd);                        
    }

};


int main(int argc, char** argv) {

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;

}