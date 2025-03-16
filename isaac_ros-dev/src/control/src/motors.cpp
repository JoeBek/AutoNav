/**
 * handles motor control speed translation commands
 */

#include <cstdint>
#include <string>
#include <utility>



std::pair<std::string,std::string> move(int32_t leftspeed, int32_t rightspeed) {

    std::string left_motor = "!G 1" + std::to_string(leftspeed) + "\r";
    std::string right_motor = "!G 2" + std::to_string(rightspeed) + "\r";

    return std::make_pair(left_motor, right_motor);

}

const std::string init() {

    return "!MG\r";
}

const std::string shutdown() {

    return "!EX\r";
}


