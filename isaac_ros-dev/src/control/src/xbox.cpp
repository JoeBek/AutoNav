#include "motors.cpp"
#include <vector>





class Xbox {


    public: 

    Xbox() : a(false), b(false), x(false), y(false), left_stick_x(0.0f),left_stick_y(0.0f), right_stick_x(0.0f),right_stick_y(0.0f)
    {


    }

    /**
     * 
     */
    std::string get_motor_speed(){

        return "";
    }

    inline void set_inputs(const std::vector<bool> &buttons, const std::vector<float> &axes){

        a = buttons[0];
        b = buttons[1];
        x = buttons[2];
        y = buttons[3];
        left_stick_x = axes[0];
        left_stick_y = axes[0];
        right_stick_x = axes[1];
        right_stick_y = axes[0];

    }



    private:

    bool a,b,y,x;
    float left_stick_x, left_stick_y, right_stick_x, right_stick_y;



    


};