#include "lib.hpp"

#define BAUDRATE 9600
#define PIN 9


String command; // global definition
Signal signal = Signal::UNKNOWN; // global definition

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

    case Signal::STOP:
      toggle_light(PIN, false);
      break;
    case Signal::SOLID:
      toggle_light(PIN, true);
      break;
    case Signal::FLASHING:
      flash_light(PIN);
      break;
    default:
      break;
  }


  
  //Serial.println(command);

  //Serial.println(message);

  delay(500);

  
}

