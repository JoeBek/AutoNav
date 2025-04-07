#ifndef LIB_HPP
#define LIB_HPP

#include <Arduino.h>


enum class Signal {

  ENCODERS,
  STOP,
  MANUAL,
  AUTONOMOUS,
  UNKNOWN
  
};


//TODO: Sree and Cassie
void bowser_drive(int wheelPin, int throttlePin, int leftRPM, int rightRPM){



}

int getLeft(String command){
  char currentChar = command.charAt(0);
  int charIndex = 0;
  String leftStr = "";

  while(currentChar != ' '){
    if(currentChar != 'L' && currentChar != ':'){
      leftStr += currentChar;
    }
    charIndex++;

    //index error check
    if(charIndex == command.length()){
      return 0;
    }

    currentChar = command.charAt(charIndex);
  }
  return leftStr.toInt();
}

int getRight(String command){
  int charIndex = 0;
  String rightStr = "";
  bool rightFlag = false;

  while(charIndex < command.length()){
    currentChar = command.charAt(charIndex);
    if(!rightFlag){
      if(currentChar == 'R'){
        charIndex++;
        rightFlag = true;
      }
    }
    else{
      rightStr += currentChar;
    }
    charIndex++;
  }
  return rightStr.toInt();
}

void flash_light(int pin_num) {

  unsigned long currentMillis = millis();

  while (millis() - currentMillis < 500) {


    
    digitalWrite(pin_num, HIGH);
    delay(500);
    digitalWrite(pin_num, LOW);
    delay(500);
  }


}

void solid_light(int pin_num){

    digitalWrite(pin_num, HIGH);

}

void toggle_light(int pin_num, bool on){



  int status = digitalRead(pin_num);

  if (on) {

    digitalWrite(pin_num, LOW);

  } 
  else {
    digitalWrite(pin_num, HIGH);

  }

  

}

Signal processCommand(String command) {

  
  if (command.length() > 0 && command.charAt(0) == 'L') {
    
    return Signal::ENCODERS;

  }
  else if (command == "stop") {
    
    return Signal::STOP;

  }
  else if (command == "AUTONOMOUS") {

    return Signal::AUTONOMOUS;

  }
  else if (command == "MANUAL") {

    return Signal::MANUAL;

  }

  return Signal::UNKNOWN;

}



#endif
