#include <chrono>
#include <cstring>
#include <cstdlib>
#include "serialib.hpp"


int get_encoder_count(const char* str) {
    const char* equalPos = strchr(str, '=');
    if (equalPos != nullptr) {
        equalPos++;
        return std::stoi(equalPos);
    } 
    else {
        std::cerr << "No '=' found in the string" << std::endl;
    }
    return 0;
}

int main(int, char**) {
    serialib motorController;
    const char *port = "/dev/ttyACM0";
    int result = motorController.openDevice(port, 115200);
    
    if (result != 1) {
        printf("Failed to open %s (error code: %d)\n", port, result);
    } else {
        printf("Successfully connected to %s\n", port);
    }

    motorController.writeString("!C 1 0\r");
    motorController.writeString("!C 2 0\r");

    while (true){
        motorController.writeString("?C 1\r");
        char recvRightEncoderBuffer[64];
        motorController.readString(recvRightEncoderBuffer, '\n', 40, 10);

        motorController.writeString("?C 2\r");
        char recvLeftEncoderBuffer[64];
        motorController.readString(recvLeftEncoderBuffer, '\n', 40, 10);

        int rightEncoderCount = get_encoder_count(recvRightEncoderBuffer);
        int leftEncoderCount = get_encoder_count(recvLeftEncoderBuffer);
        if (leftEncoderCount > 0) {
            leftEncoderCount = -leftEncoderCount;
        }
        else {
            leftEncoderCount = std::abs(leftEncoderCount);
        }

        printf("Right Encoder Count: %d\n", rightEncoderCount);
        printf("Left Encoder Count: %d\n", leftEncoderCount);
    }
}