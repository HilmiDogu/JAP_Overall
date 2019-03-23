#ifndef MICRO_MOTOR_ENCODER_H
#define MICRO_MOTOR_ENCODER_H
#include <Arduino.h>
#define LEFT_ENC_A              2
#define LEFT_ENC_B             22
#define RIGHT_ENC_A             3
#define RIGHT_ENC_B            23
#define LEFT_ENC_INT_NUMBER     0
#define RIGHT_ENC_INT_NUMBER    1
void MicroMotorEncoderInit();
void LeftEncoderTick();
void RightEncoderTick();

#endif
