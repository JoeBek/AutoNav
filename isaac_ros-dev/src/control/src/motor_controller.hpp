#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "serial.hpp"

class MotorController {
private:
    Serial* motorSerial;
    int stepSize;
    std::pair<int, int> right_turn_speeds = {-10, -10};
    std::pair<int, int> left_turn_speeds = {10, 10};
    int speed = 10;

public:
    // Constructor
    MotorController();

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
    int getLeftMotorRPM();
    int getRightMotorRPM();
};
