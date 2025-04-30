#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "serialib.hpp"

class MotorController {
private:
    int stepSize = 10;
    std::pair<int, int> right_turn_speeds = {-10, -10};
    std::pair<int, int> left_turn_speeds = {10, 10};
    int speed = 11;
    std::string comPort;


public:
    // Constructor
    MotorController();
    serialib motorSerial;
    // configuration
    char configure(const char * port);

    // Moveeeeeee
    void forward();
    void backward();
    void turnLeft();
    void turnRight();
    void move(float right_speed, float left_speed);
    void stop();
    void shutdown();

    // Get and set
    void setStepSize(int size);
    int getStepSize();
    void setSpeed(int s);
    int getSpeed();
    int  getLeftEncoderCount();
    int getRightEncoderCount();
    int getLeftRPM();
    int getRightRPM();
};
