#include <PID_v1.h>
int PWM1=6;
int PWM2=7;
double Input,Output,Setpoint, Error;
double Kp1=0.1,Ki1=0.00, Kd1=0.00;
double Kp2=0.1, Kd2=0.00, Ki2=0.00;
int Outputmap1=0;
int Outputmap2=0;
PID myPID(&Input, &Output, &Setpoint, Kp1, Ki1, Kd1, DIRECT); 


void setup()
{
  //initialize the variables we're linked to
  Input = 90;
  Setpoint= analogRead(0);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
 myPID.SetSampleTime(125);
}



void loop() {
  // put your main code here, to run repeatedly:
  
Input = 90;
Setpoint= analogRead(1);
Error=Setpoint-Input;

if(Error<50){
myPID.SetTunings(Kp1, Ki1, Kd1);
}
else if(Error>50){
  myPID.SetTunings(Kp2, Ki2, Kd2);
}
 myPID.Compute();
 
  Outputmap1=map(Output,0,1023,40,70);
  Outputmap2=map(-Output,0,1023,40,70);
  
analogWrite(PWM1,Outputmap1);
analogWrite(PWM2,Outputmap2);
}
