#include <iostream>
#include <string>
#include <cmath>

class Xbox {

public:
    Xbox();

    enum Command {
      STOP = 0,
      MOVE = 1,
      SPEED_UP = 2,
      SPEED_DOWN = 3,
      NONE = 4,
      CHANGE_MODE = 5,
      GET_CURRENT = 6
  };

    struct CommandData {
      Command command;
      float left_motor_speed;
      float right_motor_speed;
  };

    CommandData calculateCommand();
    
    void set_b(bool state);
    void set_x(bool state);
    void set_y(bool state);
    void set_right_bumper(bool state);
    void set_left_bumper(bool state);

    void set_left_stick_x(float pos);
    void set_left_stick_y(float pos);
    void set_right_stick_x(float pos);
    void set_right_stick_y(float pos);

    void adjust_joysticks();

private:
    bool b_button_state;
    bool x_button_state;
    bool y_button_state;
    bool right_bumper_state;
    bool left_bumper_state;

    float left_stick_x_pos;
    float left_stick_y_pos;
    float right_stick_x_pos;
    float left_stick_y_pos;


    bool TANKDRIVE = true;
};
