#ifndef SHARP_5CM_H
#define SHARP_5CM_H
#include <Arduino.h>
#define SHARP_5CM_FRONT 21
#define SHARP_5CM_FRONT_INT_NUMBER  2
#define SHARP_5CM_BACK  15
void Sharp_5cm_Init();
bool Sharp_5cm_Back_Check();
void Approach_5cm_Front_Event();
#endif
