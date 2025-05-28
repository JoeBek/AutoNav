#include "serialib.cpp" 

#include <iostream> 

#include <string> 

#include <sstream> 

#include <vector> 

 

// GPS Serial Port 

#define SERIAL_PORT "\\\\.\\COM10" 

 

void init_gps(serialib &gps) { 

  char opened = gps.openDevice("/dev/ttyTHS1", 9600); 

 

  // Check if Serial Connection was successful 

  if (opened != 1) { 

    std::cerr << "Failed to open serial port: " << (int)opened << '\n' << std::endl; 

    return; 

  } 

  std::cout << "Successful connection to " << SERIAL_PORT << '\n' << std::endl; 

} 

 

int main() { 

  serialib gpsSerial; 

  init_gps(gpsSerial); 

 

  // Uncomment this stuff out when figuring out the GPS Parsing stuff 

  while (true) { 

    char gpsBuffer[1024] = {}; 

    int bytesRead = gpsSerial.readString(gpsBuffer, '\n', 1023, 1000); 

     

    std::string gpsData; 

    if (bytesRead > 0) { 

      std::string message(gpsBuffer); 

      if (!message.empty()) { 

        std::cout << "Received message: " << message << std::endl; 

        gpsData = message; 

      } 
    }  

    else { 

      std::cerr << "No data received within timeout" << std::endl; 

    } 

  } 

} 
