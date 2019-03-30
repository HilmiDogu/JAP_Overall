#include "Sharp_5cm.h"
void Sharp_5cm_Init(){
    pinMode(SHARP_5CM_FRONT, INPUT);
    pinMode(SHARP_5CM_BACK, INPUT);
    attachInterrupt(SHARP_5CM_FRONT_INT_NUMBER, Approach_5cm_Front_Event, CHANGE);    
}

bool Sharp_5cm_Back_Check(){    //Returns true if an object is approached closer than or equal to 5 cm.
    return !digitalRead(SHARP_5CM_BACK);
}
