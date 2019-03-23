#ifndef TB6612_H
#define TB6612_H
#include <Arduino.h>
#define FORWARD 1
#define BACKWARD 0
#define LEFT_IN_1   5
#define LEFT_IN_2   6
#define LEFT_PWM    4
#define RIGHT_IN_1  8
#define RIGHT_IN_2  9
#define RIGHT_PWM   7
#define STANDBY     10
class TB6612{
  public:
    TB6612();
    TB6612(int, int, int, int, int, int, int);//Constructor
    void AngleInput(float);
    void LeftMotorWrite(int, bool);
    void RightMotorWrite(int, bool);
    void Stop(int);
    void Resume();
  private:
    int stdby, pwma, pwmb, ain1, ain2, bin1, bin2; //Pin numbers
    int rmspeed, lmspeed;
    bool motor_dir;
    float angle_in;
};

#endif
