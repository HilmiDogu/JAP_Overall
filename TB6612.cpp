#include "TB6612.h"
TB6612::TB6612(){
  angle_in  = 0.0;
  motor_dir = FORWARD;
}
TB6612::TB6612(int STDBY, int PWMA, int PWMB, int AIN1, int AIN2, int BIN1, int BIN2){
  angle_in  = 0.0;
  stdby = STDBY;
  pwma = PWMA;
  pwmb = PWMB;
  ain1 = AIN1;
  ain2 = AIN2;
  bin1 = BIN1;
  bin2 = BIN2;
  pinMode(STDBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  digitalWrite(STDBY, HIGH);
  
  motor_dir = FORWARD;
}
void TB6612::AngleInput(float ang){
  angle_in = ang;  
}
void TB6612::LeftMotorWrite(int lmw, bool dir){
    if(dir == true){
        digitalWrite(bin1, HIGH);
        digitalWrite(bin2, LOW);
    }
    else{
        digitalWrite(bin1, LOW);
        digitalWrite(bin2, HIGH);  
    }
    lmspeed = lmw;
    analogWrite(pwmb, lmw);
}
void TB6612::RightMotorWrite(int rmw, bool dir){
    if(dir == true){
        digitalWrite(ain1, HIGH);
        digitalWrite(ain2, LOW);
    }
    else{
        digitalWrite(ain1, LOW);
        digitalWrite(ain2, HIGH);  
    }
    rmspeed = rmw;
    analogWrite(pwma, rmw);
}
void TB6612::Stop(int dly){
    if(dly <= 0){
        digitalWrite(stdby, LOW);   
    }
    else{
        digitalWrite(stdby, LOW);
        delay(dly);
        digitalWrite(stdby, HIGH);    
    }
}
void TB6612::Resume(){
    digitalWrite(stdby, HIGH);    
}
