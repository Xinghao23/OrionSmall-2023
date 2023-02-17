#include "main.h"


//Cata



//Roller

void rollerChange(char colour){
    rollerSense.set_led_pwm(100);
    pros::c::optical_rgb_s_t rollerColour;
    int timeStart = pros::c::millis();
    if(colour == 'b'){
        rollerColour = rollerSense.get_rgb(); 
        while(rollerColour.red < 10000 && timeStart < pros::c::millis()+2000)
        {
            intake = 60;
            rollerColour = rollerSense.get_rgb();
        }
    }
    intake = 0;
    rollerSense.set_led_pwm(0);
}




//Helper

void setCata(int motorPower){
    rightCata.move(motorPower);
    leftCata.move(motorPower);
}