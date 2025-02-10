#include "motors.cpp"





class Xbox {


    public: 

    Xbox() : a(false), b(false), x(false), y(false), left_stick(0.0f), right_stick(0.0f)
    {


    }

    /**
     * 
     */
    std::string get_motor_speed(){

        return "";
    }



    private:

    bool a,b,y,x;
    float left_stick, right_stick;




    


}