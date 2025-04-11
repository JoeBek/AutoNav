#ifndef LIB_HPP
#define LIB_HPP

#include <Arduino.h>


enum class Signal {

  STOP,
  FLASHING,
  SOLID,
  UNKNOWN
  
};


void flash_light(int pin_num) {

  unsigned long currentMillis = millis();

  while (millis() - currentMillis < 500) {


    
    digitalWrite(pin_num, HIGH);
    delay(500);
    digitalWrite(pin_num, LOW);
    delay(500);
  }


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

  
  
  if (command == "stop") {
    
    return Signal::STOP;

  }
  else if (command == "solid") {
    return Signal::SOLID;

  }
  else if (command == "blink") {

    
    return Signal::FLASHING;

  }

  return Signal::UNKNOWN;

}



#endif
