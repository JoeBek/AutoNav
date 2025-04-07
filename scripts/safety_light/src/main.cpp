#include "lib.hpp"

#define BAUDRATE 9600
#define LIGHT_PIN 9

//TODO: Update these pins
#define WHEEL_PIN 10
#define THROTTLE_PIN 11


String command; // global definition
Signal signal = Signal::UNKNOWN; // global definition
int leftRPM; // global definition
int rightRPM; // global definition

void setup(){

  Serial.begin(BAUDRATE);
  Serial.setTimeout(1);

  pinMode(9, OUTPUT);
  pinMode(3, OUTPUT);

  digitalWrite(3, HIGH);
  digitalWrite(9, HIGH);

  
 
}



void loop(){


  if (Serial.available()) {

    command = Serial.readStringUntil('\n');
    signal = processCommand(command);
    Serial.println("new command is: " + command);

  }
  



  switch(signal) {
    case Signal::ENCODERS:
      leftRPM = getLeft(command);
      rightRPM = getRight(command);
      bowser_drive(WHEEL_PIN, THROTTLE_PIN, leftRPM, rightRPM);
      break;
    case Signal::STOP:
      toggle_light(LIGHT_PIN, false);
      break;
    case Signal::MANUAL:
      solid_light(LIGHT_PIN);
      break;
    case Signal::AUTONOMOUS:
      flash_light(LIGHT_PIN);
      break;
    default:
      break;
  }


  
  //Serial.println(command);

  //Serial.println(message);

  delay(500);

  
}

