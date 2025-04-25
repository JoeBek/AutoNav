
#include <rclcpp/rclcpp.hpp>
#include "serialib.hpp"
#include "xbox.hpp"
#include "motor_controller.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "autonav_interfaces/msg/encoders.hpp"
#include "autonav_interfaces/msg/gps_data.hpp"  
#include "autonav_interfaces/srv/configure_control.hpp"
#include <queue>
#include <iostream>
#include <string>
#include <cmath>


class ControlNode : public rclcpp::Node {

    public:


    ControlNode() 
      : Node("control_node")
       {

        // topic names
        this->declare_parameter("controller_topic", "joy");
        this->declare_parameter("encoder_topic", "encoders");
        this->declare_parameter("gps_topic", "gps");
        this->declare_parameter("path_planning_topic", "cmd_vel");
        
        // serial ports
        this->declare_parameter("motor_port", "/dev/ttyACM0");
        this->declare_parameter("gps_port", "/dev/ttyTCU0");
        this->declare_parameter("arduino_port", "/dev/ttyACM2");

        configure_server = this->create_service<autonav_interfaces::srv::ConfigureControl>
             ("configure_control", std::bind(&ControlNode::configure, this, std::placeholders::_1, std::placeholders::_2));

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
    //configuration server 
    rclcpp::Service<autonav_interfaces::srv::ConfigureControl>::SharedPtr configure_server;

    // publisher for gps data
    //rclcpp::Publisher<autonav_interfaces::msg::GpsData>::SharedPtr gpsPub;
    //rclcpp::TimerBase::SharedPtr gps_timer_;

    // publisher for encoder values
   // rclcpp::Publisher<autonav_interfaces::msg::Encoders>::SharedPtr encodersPub;
   // rclcpp::TimerBase::SharedPtr encoder_timer_;

    // subscription for Nav2 pose
    //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pathPlanningSub;

    bool currX = false;
    bool prevX = false;

    void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        currX = joy_msg->buttons[3];

        // Detect rising edge
        if (!prevX && currX) {
            autonomousMode = !autonomousMode;

            if (autonomousMode) {
                char mode[12] = "AUTONOMOUS\n";
                arduinoSerial.writeString(mode);
            } else {
                char mode[8] = "MANUAL\n";
                arduinoSerial.writeString(mode);
            }
        }

        prevX = currX;

        if(!autonomousMode){
            controller.set_b(joy_msg->buttons[1]);
            controller.set_x(joy_msg->buttons[3]);
            controller.set_y(joy_msg->buttons[2]);

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
        }
    }

    void publish_encoder_data() {
        autonav_interfaces::msg::Encoders encoder_msg;
        encoder_msg.left_motor_rpm = motors.getLeftRPM();
        encoder_msg.right_motor_rpm = motors.getRightRPM();
        encoder_msg.left_motor_count = motors.getLeftEncoderCount();
        encoder_msg.right_motor_count = motors.getRightEncoderCount();

        std::string arduinoRPMs = "L:";
        arduinoRPMs += encoder_msg.left_motor_rpm;
        arduinoRPMs += " R:";
        arduinoRPMs += encoder_msg.right_motor_rpm;
        arduinoRPMs += "\n";
        arduinoSerial.writeString(arduinoRPMs.c_str());

        //encodersPub->publish(encoder_msg);
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
            gps_msg.altitude = std::stof(tokens[5]);
  	    }
	    catch (const std::exception & e) {
		    RCLCPP_ERROR(this->get_logger(), "GPS string parsing failed: %s", e.what());
	    }

            //gpsPub->publish(gps_msg);
        }
    }

    //void path_planning_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

        /*if (autonomousMode) {
            currPose.positionX = msg->pose.position.x,
            currPose.positionY = msg->pose.position.y,
            currPose.positionZ = msg->pose.position.z;

            currPose.orientationX = msg->pose.orientation.x,
            currPose.orientationY = msg->pose.orientation.y,
            currPose.orientationZ = msg->pose.orientation.z;
            currPose.orientationW = msg->pose.orientation.w;

            double absoluteYaw = currPose.getYawFromQuaternion();
            double relativeYaw = ........
            double relativeX = ........
            double relativeY = ........
            double initialTurnAngle = std::atan2(relativeY, relativeX) * (180 / M_PI);
            double totalDistance = std::sqrt((relativeX * relativeX) + (relativeY * relativeY))

            while(motors.getLeftEncoderCount() < ((initialTurnAngle / 360) * 67800)){
                motors.move(-10, 10);
            }

            //1.275 is 50.2 inches in meters
            if(totalDistance > 1.275){
                while(motors.getLeftEncoderCount() < ((totalDistance / 1.275) * 80000)){
                    motors.move(10, 10);
                }
            }
            else {
                while(motors.getLeftEncoderCount() < ((totalDistance / 1.275) * 67800)){
                    motors.move(10, 10);
                }
            }
        }*/
    //}


    void init_serial_arduino(const char * arduino_port) {

        char ret;
        ret = arduinoSerial.openDevice(arduino_port, 9600);
        
        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Arduino serial error: %s", arduinoSerial.error_map.at(ret).c_str());
        }

        char mode[8] = "MANUAL\n";
        arduinoSerial.writeString(mode);
                   
    }

    void init_serial_gps(const char * gps_port) {

        char ret;
        ret = gpsSerial.openDevice(gps_port, 115200);
        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "GPS serial error: %s", gpsSerial.error_map.at(ret).c_str());
        }

        char gpsStartCmd[32] = "log bestposa ontime 2\r\n";
        gpsSerial.writeString("unlogall\r\n");
        gpsSerial.writeString(gpsStartCmd);     

    }


    void configure(const std::shared_ptr<autonav_interfaces::srv::ConfigureControl::Request> request, 
                         std::shared_ptr<autonav_interfaces::srv::ConfigureControl::Response> response) {
        

        // configure serial
        std::string motor_port = this->get_parameter("motor_port").as_string();
        std::string arduino_port = this->get_parameter("arduino_port").as_string();
        std::string gps_port = this->get_parameter("gps_port").as_string();


        if (request->arduino) {
            init_serial_arduino(arduino_port.c_str());
        }
        if (request->motors) {
            char ret = motors.configure(motor_port.c_str());
            if (ret != 1) {
                RCLCPP_ERROR(this->get_logger(), "Motor serial error: %s", gpsSerial.error_map.at(ret).c_str());
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Motor serial connection success!");
            }
        }
        if (request->gps) {
            init_serial_gps(gps_port.c_str());
        }
                
        // configure topics
        std::string controller_topic = this->get_parameter("controller_topic").as_string();
        std::string encoder_topic = this->get_parameter("encoder_topic").as_string();
        std::string gps_topic = this->get_parameter("gps_topic").as_string();
        std::string path_planning_topic = this->get_parameter("gps_topic").as_string();

        //XBOX SUB
        controllerSub = this->create_subscription<sensor_msgs::msg::Joy>(
            controller_topic, 10, std::bind(&ControlNode::joystick_callback, this, std::placeholders::_1));

        //NAVIGATION ENCODER PUB
        //encodersPub = this->create_publisher<autonav_interfaces::msg::Encoders>(encoder_topic, 10);
        
       /* encoder_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControlNode::publish_encoder_data, this)
        );*/
        
        
        //GPS PUB

        if (request->gps) {

            /*gpsPub = this->create_publisher<autonav_interfaces::msg::GpsData>(gps_topic, 10);

            gps_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&ControlNode::publish_gps_data, this)
            );*/
    
        }
       
        //PATH PLANNING SUB
        /*pathPlanningSub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ControlNode::path_planning_callback, this, std::placeholders::_1));
              */

        response->ret = 0;
        std::string leftMotorCommand = "!C 1 0\r";
        std::string rightMotorCommand = "!C 2 0 \r";
        motors.motorSerial.writeString(leftMotorCommand.c_str());
        motors.motorSerial.writeString(rightMotorCommand.c_str());
        /*while(motors.getLeftEncoderCount() < 122500){ ccw
            move(-10, 10);
            std::this_thread::sleep_for(std::chrono::milliseconds(1))
        }
        stop();*/


        
        /*const double PI = 3.14159265358979323846;
        double relativeYaw = 270;
        double relativeX = 3;
        double relativeY = 4;
        double initialTurnAngle = std::atan2(relativeY, relativeX) * (180 / PI);
        double totalDistance = std::sqrt((relativeX * relativeX) + (relativeY * relativeY));

        int totalEncoderCount = motors.getLeftEncoderCount();

        while(static_cast<double>(motors.getLeftEncoderCount()) < (static_cast<double>(initialTurnAngle / 360) * 122500)){
            motors.move(-10, 10);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        motors.stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        totalEncoderCount = motors.getLeftEncoderCount();
        //1.275 is 50.2 inches in meters
        if(totalDistance > 1.275){
            while(static_cast<double>(motors.getLeftEncoderCount()) < ((static_cast<double>((totalDistance / 1.275) * 76000)) + totalEncoderCount)){
                motors.move(10, 10);
                //std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        else {
            while(static_cast<double>(motors.getLeftEncoderCount()) < ((static_cast<double>((totalDistance / 1.275) * 67800)) + totalEncoderCount)){
                motors.move(10, 10);
                //std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        motors.stop();

        std::string stoppp = "!EX\r";
        std::string starttt = "!MG\r";
        std::string clear = "# C\r";
        std::string leftStop = "!MS 1\r";
        std::string rightStop = "!MS 2\r";

        int count = 0;*/
        /*while(motors.getLeftEncoderCount() < 96000){
          
            motors.move(10, 10);
                 //motors.motorSerial.writeString(clear.c_str());
            //std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        //motors.motorSerial.writeString(clear.c_str());
        motors.stop();

        count = 0;
        while(motors.getLeftEncoderCount() > 0){
            
            motors.move(-10, -10);
                 //motors.motorSerial.writeString(clear.c_str());
            
            //std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        motors.stop();*/
    }

};


int main(int argc, char** argv) {

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;

}
