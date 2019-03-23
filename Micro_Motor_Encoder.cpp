#include "Micro_Motor_Encoder.h"
long leftTickCount;
long rightTickCount;
void MicroMotorEncoderInit(){
    pinMode(LEFT_ENC_A,  INPUT);
    pinMode(LEFT_ENC_B,  INPUT);
    pinMode(RIGHT_ENC_A, INPUT);
    pinMode(RIGHT_ENC_B, INPUT);

    attachInterrupt(LEFT_ENC_INT_NUMBER, LeftEncoderTick, CHANGE);
    attachInterrupt(RIGHT_ENC_INT_NUMBER, RightEncoderTick, CHANGE);
}

void LeftEncoderTick(){
    if (digitalRead(LEFT_ENC_A) == HIGH) {
        if (digitalRead(LEFT_ENC_B) == LOW) {
          leftTickCount++;
        } 
        else {
            leftTickCount--;
        }
    } 
    else {
        if (digitalRead(LEFT_ENC_B) == LOW) {
            leftTickCount--;
        } 
        else {
            leftTickCount++;
    }
  }
}

void RightEncoderTick() {
  if (digitalRead(RIGHT_ENC_A) == HIGH) {
    if (digitalRead(RIGHT_ENC_B) == LOW) {
      rightTickCount--;
    } else {
      rightTickCount++;
    }
  } else {
    if (digitalRead(RIGHT_ENC_B) == LOW) {
      rightTickCount++;
    } else {
      rightTickCount--;
    }
  }
}
