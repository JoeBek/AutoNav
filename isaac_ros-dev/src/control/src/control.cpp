
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
#include <string>


class ControlNode : public rclcpp::Node {

    public:


    ControlNode() 
      : Node("control_node")
       {

        initialize_serial_connections();

        //XBOX SUB
        controllerSub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ControlNode::joystick_callback, this, std::placeholders::_1));

        //ENCODER PUB
        encodersPub = this->create_publisher<autonav_interfaces::msg::Encoders>("encoder_topic", 10);

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
        /*pathPlanningSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/amcl_pose", 10, std::bind(&ControlNode::path_planning_callback, this, std::placeholders::_1));*/
    }


    private:
    serialib arduinoSerial;
    serialib gpsSerial;
    Xbox controller;
    MotorController motors;
    //Autonomous currPose;

    bool autonomousMode = false;


    // subscription for joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controllerSub;

    // publisher for gps data
    rclcpp::Publisher<autonav_interfaces::msg::GpsData>::SharedPtr gpsPub;
    rclcpp::TimerBase::SharedPtr gps_timer_;

    // publisher for encoder values
    rclcpp::Publisher<autonav_interfaces::msg::Encoders>::SharedPtr encodersPub;
    rclcpp::TimerBase::SharedPtr encoder_timer_;

    // subscription for Nav2 pose
    //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pathPlanningSub;

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
        
            if(controller.switchMode()){
                autonomousMode = false;
                char mode[8] = "MANUAL\n";
                arduinoSerial.writeString(mode);
            }
        }
        
    }

    void publish_encoder_data() {
        autonav_interfaces::msg::Encoders encoder_msg;
        encoder_msg.left_motor_rpm = std::to_string(motors.getLeftRPM() / 20);
        encoder_msg.right_motor_rpm = std::to_string(motors.getRightRPM() / 20);
        encoder_msg.left_motor_count = motors.getLeftEncoderCount();
        encoder_msg.right_motor_count = motors.getRightEncoderCount();

        std::string arduinoRPMs = "L:";
        arduinoRPMs += encoder_msg.left_motor_rpm;
        arduinoRPMs += " R:";
        arduinoRPMs += encoder_msg.right_motor_rpm;
        arduinoRPMs += "\n";
        arduinoSerial.writeString(arduinoRPMs.c_str());

        encodersPub->publish(encoder_msg);
    }

    void publish_gps_data() {
        autonav_interfaces::msg::GpsData gps_msg;
        char gpsBuffer[1024] = {};
        gpsSerial.readString(gpsBuffer, '\n', 1023, 1000);


        std::string message(gpsBuffer);
        std::istringstream iss(message);
        std::string word;
        std::vector<std::string> tokens;

        while (iss >> word) {
            tokens.push_back(word);
        }
        if (tokens.size() == 22) {

            //std::cout << gpsBuffer << std::endl;

	    try {

		    gps_msg.latitude = std::stof(tokens[3]);
		    gps_msg.longitude = std::stof(tokens[4]);
  	    }
	    catch (const std::exception & e) {
		    RCLCPP_ERROR(this->get_logger(), "GPS string parsing failed: %s", e.what());
	    }

            gpsPub->publish(gps_msg);
        }
    }

    /*void path_planning_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

        if (autonomousMode) {
            currPose.positionX = msg->pose.position.x,
            currPose.positionY = msg->pose.position.y,
            currPose.positionZ = msg->pose.position.z;

            currPose.orientationX = msg->pose.orientation.x,
            currPose.orientationY = msg->pose.orientation.y,
            currPose.orientationZ = msg->pose.orientation.z;
            currPose.orientationW = msg->pose.orientation.w;

            double yaw = currPose.getYawFromQuaternion();
        }
    }*/


    void initialize_serial_connections() {
        // Open all serial connections on startup
        
        char arduinoStatus = arduinoSerial.openDevice("/dev/ttyACM0", 9600);
        if (arduinoStatus != 1){
            printf ("Unsuccessful connection to Arduino\n");
          }
        else{
            printf ("Successful connection to Arduino\n");
        }

        char mode[8] = "MANUAL\n";
        arduinoSerial.writeString(mode);

        char gpsStatus = gpsSerial.openDevice("/dev/ttyACM1", 115200);
        if (gpsStatus != 1){
            printf ("Unsuccessful connection to GPS\n");
        }
        else{
            printf ("Successful connection to GPS\n");
        }
        char gpsStartCmd[32] = "log bestposa ontime 2\r\n";
        gpsSerial.writeString("unlogall\r\n");
        gpsSerial.writeString(gpsStartCmd);                        
    }

};


int main(int argc, char** argv) {

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;

}
