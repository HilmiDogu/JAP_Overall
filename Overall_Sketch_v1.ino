/*
 * 100 nF-500 nF capacitor is required for proper communication with Raspberry
 * Pi and Arduino Mega board via USB Cable.
 * Disconnection of Arduino Power Input Cable speeds up trying process.
  This sketch includes all of the components of the robot such as
  Wireless Communication via ESP8266
  Communication with Raspberry Pi
  Motor Drive
  PID
  5 cm detection via Distance Sensors
  Encoder PID Parameters:
  Kp = -0.6
  Ki = -0.1
  Kd = -0.05
  Or Kp = -0.9 
*/

/*
 * Optimal PID equation related to elliptical track could be written 
 * by using tangent of the angle and multiplying with some constant.
 */
#include "Handshake.h"
#include "TB6612.h"
#include "Micro_Motor_Encoder.h"
#include "Sharp_5cm.h"
#include <PID_v1.h>
#define PI      3.141592
static HANDSHAKE_STATE Current_State = NORMAL;
static CATCH_STATE Current_Catch_State = INIT;
static TB6612 Motor(STANDBY, RIGHT_PWM, LEFT_PWM, RIGHT_IN_1, RIGHT_IN_2, LEFT_IN_1, LEFT_IN_2);
extern String rcv;
extern String remoteID;
extern char rmtID[2];
extern long leftTickCount;
extern long rightTickCount;
static unsigned int lmpwmArray[5], rmpwmArray[5], lmpwmGuess, rmpwmGuess;
int lmpwm, rmpwm;
int lmpwmbase   = 60;
int pwmbaseadd  = 0;
int lmpwmadd    = 0;
int rmpwmbase   = 60;
int rmpwmadd    = 0;
int outLimit    = 90;
String angleString = "";    //  String to hold incoming data
int angleValue;
bool angleArrived = false;  //  whether the string is complete

double Input,Output,Setpoint, Error, setmap;
double diff1Error = 0, diff2Error = 0, diff2nd = 0, prev1Error = 0, prev2Error = 0;
double Kp_add, Ki_add, Kd_add;
double Kpbase=0.1, Kp=0.4,  Kibase=0, Kd=1.5 , Ki=0.0, Kdbase=0, Kd2nd;
//double Kp2=0.65, Ki2=0.15, Kd2=0.12;
int Outputmap1=0;
int Outputmap2=0;
float a4=0.8,a2=0.8,a3=0.2,a1=0.2,a0=0.1;
float a_avg = abs(a4)+abs(a3)+abs(a2)+abs(a1)+abs(a0);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); 
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);  
    Serial2.begin(115200);
    
    Sharp_5cm_Init();
    MicroMotorEncoderInit();
    //WiFi_Init();
    Input = 90; 
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(125);
    Motor.Stop(2000);

}
char inChar;
void loop(){
   //State 0
        if(Current_State == NORMAL && Current_Catch_State == INIT){
            WiFi_LED_Write(INIT, NORMAL);
//            Motor Drive, Raspberry Pi communication and PID control here. (Maybe Distance Sensor Reading)
            
            while(!Serial2.available());
                while(Serial2.available()){
                inChar = Serial2.read();
                angleString += inChar;
                if (inChar == '\n') {
                    angleArrived = true;
                    if(DEBUG_MODE) Serial.println(angleString);
                }   
            }
            if (angleArrived){      
                if(angleString.indexOf("NOP")){
                    //if(DEBUG_MODE) Serial.println("Not nop");
                    angleValue = angleString.toInt();
                    Setpoint = (double)angleValue;
                    if(DEBUG_MODE) Serial.println(Setpoint);
                    setmap = map(Setpoint, 40,140, -90, 90);
                    pwmbaseadd = 130*(sin(PI*setmap/180))*(sin(PI*setmap/180));
                    Error       = Setpoint - Input;
                    diff1Error = Error - prev1Error;
                    diff2Error = prev1Error - prev2Error;
                    diff2nd = diff1Error - diff2Error;
                    prev1Error = Error;
                    prev2Error = prev1Error;
                    Output = Kp*Error + Kd*diff1Error + Kd2nd*diff2nd;
                    Kp=0.75;
                    Kd=1.00;
                    Kd2nd=0.3;
                    lmpwmadd = map(Output, -90,90, -40, 40);
                    lmpwmArray[0] = lmpwmArray[1];
                    lmpwmArray[1] = lmpwmArray[2];
                    lmpwmArray[2] = lmpwmArray[3];
                    lmpwmArray[3] = lmpwmArray[4];
                    lmpwmArray[4] = lmpwmbase - lmpwmadd + pwmbaseadd;
                    if(abs(Error) > 50) lmpwmArray[4] = (lmpwmArray[4] * a4 + lmpwmArray[3] * a3 + lmpwmArray[2] * a2 + lmpwmArray[1] * a1 + lmpwmArray[0] * a0)/a_avg; 

                    rmpwmArray[0] = rmpwmArray[1];
                    rmpwmArray[1] = rmpwmArray[2];
                    rmpwmArray[2] = rmpwmArray[3];
                    rmpwmArray[3] = rmpwmArray[4];
                    rmpwmArray[4] = rmpwmbase + lmpwmadd + pwmbaseadd;
                    if(abs(Error) > 50)rmpwmArray[4] = (rmpwmArray[4] * a4 + rmpwmArray[3] * a3 + rmpwmArray[2] * a2 + rmpwmArray[1] * a1 + rmpwmArray[0] * a0)/a_avg;
//                    Motor.LeftMotorWrite(lmpwmArray[4], FORWARD);
//                    Motor.RightMotorWrite(rmpwmArray[4],  FORWARD);
                    angleString = "";
                    angleArrived = false; 
//                    if(DEBUG_MODE) Serial.println(lmpwm - Output);
//                    if(DEBUG_MODE) Serial.println(rmpwm + Output);
                }
                else{
                    if(DEBUG_MODE) Serial.println("Nop");
                    Motor.LeftMotorWrite(0, FORWARD);
                    Motor.RightMotorWrite(0,  FORWARD);                   
                    angleString = "";
                    angleArrived = false; 
                }
            }
        }
        if (ESP_IfDataArrived() && Current_State == NORMAL) { // If character is came from ESP8266, it is catched by another robot
            String s = Serial1.readString();
            String ss = s.substring(13);
            String ID = s.substring(11,13);
            if(DEBUG_MODE){
                Serial.println(s);
                Serial.println(ss);
                Serial.println(ID);
                Serial.println(ID.equals(remoteID));
            }
            if (ss.indexOf("00") >= 0 && ID.equals(remoteID)) if(DEBUG_MODE) Console_WriteCharArray("Catch request(0400) is received.");
            else if(!ID.equals(remoteID)){
                if(DEBUG_MODE) Console_WriteCharArray("Wrong ID.");
               
            }
            Current_Catch_State = CATCHED;
            Current_State = CATCH_REQUEST;
        }
      //State 1
        if (Current_Catch_State == CATCHER && Current_State == CATCH_REQUEST) {
            if (ESP_IfDataArrived()) {
              String s = Serial1.readString();
              String ss = s.substring(13);
              String ID = s.substring(11,13);
              if(DEBUG_MODE){
                  Serial.println(s);
                  Serial.println(ss);
                  Serial.println(ID);
                  Serial.println(ID.equals(remoteID));
              }
              if (ss.indexOf("01") >= 0 && ID.equals(remoteID)) {
                  if(DEBUG_MODE) Console_WriteCharArray("Catch is confirmed(0401).");
                  WiFi_LED_Write(CATCHER, CATCH_CONFIRM);
                  Current_State = CATCH_CONFIRM;
              }
              else if (ss.indexOf("11") >= 0 && ID.equals(remoteID)) {
                  if(DEBUG_MODE) Console_WriteCharArray("Catch is not confirmed(0411).");
                  WiFi_LED_Write(CATCHER, CATCH_REJECT);
                  delay(1000);
                  Current_Catch_State = INIT;
                  Current_State = NORMAL;
                 
              }
        }
      }
      else if (Current_Catch_State == CATCHED && Current_State == CATCH_REQUEST) {
        bool approachedFromBack = Sharp_5cm_Back_Check();
        if (approachedFromBack == true) {
          if(DEBUG_MODE) Console_WriteCharArray("Confirm for catch(1001) sent.");
          ESP_WriteCharArray("AT+CIPSEND=0,4");
          ESP_WaitForResponse();
          rcv = Serial1.readString();
          WiFi_LED_Write(CATCHER, CATCH_CONFIRM);
          if (rcv.indexOf((char*)"OK")) ESP_WriteCharArray("1001");
          Current_State = CATCH_RESULT;
        }
        else{
          if(DEBUG_MODE) Console_WriteCharArray("No confirm for catch(1011) sent.");
          ESP_WriteCharArray("AT+CIPSEND=0,4");
          ESP_WaitForResponse();
          rcv = Serial1.readString();
          if (rcv.indexOf((char*)"OK")) ESP_WriteCharArray("1011");
          ESP_WaitForResponse();
          rcv = Serial1.readString();
          WiFi_LED_Write(CATCHER, CATCH_REJECT);
          Current_State = NORMAL;
          Current_Catch_State = INIT;
        }
      }
        //State 2
        else if(Current_Catch_State == CATCHER && Current_State == CATCH_RESULT){
            if(DEBUG_MODE) Console_WriteCharArray("Stop command(1010) sent.");
            ESP_WriteCharArray("AT+CIPSEND=0,4");
            ESP_WaitForResponse();
            rcv = Serial1.readString();
            if (rcv.indexOf((char*)"OK")) ESP_WriteCharArray("1010");
            ESP_WaitForResponse();
            rcv = Serial1.readString();
            WiFi_LED_Write(CATCHER, CATCH_RESULT);
            delay(10000);
            WiFi_LED_Write(INIT, NORMAL);
            while(1);//End of the handshake
        }
        else if(Current_Catch_State == CATCHED && Current_State == CATCH_RESULT){
            ESP_WaitForResponse();
            Serial1.readString();
            ESP_WaitForResponse();
            String s = Serial1.readString();
            String ss = s.substring(13);
            String ID = s.substring(11,13);
            if(DEBUG_MODE){
              Serial.println(s);
              Serial.println(ss);
              Serial.println(ID);
              Serial.println(ID.equals(remoteID));
            }
            if (ss.indexOf("10") >= 0 && ID.equals(remoteID)) if(DEBUG_MODE) Console_WriteCharArray("Stop command(0410) is received.");
            WiFi_LED_Write(CATCHER, CATCH_RESULT);
            delay(10000);
            WiFi_LED_Write(INIT, NORMAL);
            Motor.Stop(0);
            while(1);//End of the handshake
        }
}

void Update_PWM_Arrays(){
    lmpwmArray[0] = lmpwmArray[1];
    lmpwmArray[1] = lmpwmArray[2];
    lmpwmArray[2] = lmpwmbase - Output;
    rmpwmArray[0] = rmpwmArray[1];
    rmpwmArray[1] = rmpwmArray[2];
    rmpwmArray[2] = rmpwmbase + Output;
    lmpwmGuess    = 3*(lmpwmArray[2] - lmpwmArray[1])   + lmpwmArray[0];
    rmpwmGuess    = 3*(rmpwmArray[2] - rmpwmArray[1])   + rmpwmArray[0];
}
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void Approach_5cm_Front_Event(){
    if(!digitalRead(SHARP_5CM_FRONT) && Current_State == NORMAL){
        ESP_WriteCharArray("AT+CIPSEND=0,4");
        ESP_WaitForResponse();
        rcv = Serial1.readString();
        if (rcv.indexOf((char*)"OK")) ESP_WriteCharArray("1000");
        if(DEBUG_MODE) Console_WriteCharArray("Catch request(1000) is sent.");
        WiFi_LED_Write(CATCHER, CATCH_REQUEST);
        Current_Catch_State = CATCHER;
        Current_State = CATCH_REQUEST; 
    }    
}
//void serialEvent() {
//  while (Serial2.available()) {
//    // get the new byte:
//    char inChar = (char)Serial2.read();
//    
//    // add it to the angleString:
//    
//    angleString += inChar;
//    // if the incoming character is a newline, set a flag so the main loop can
//    // do something about it:
//    if (inChar == '\n') {
//      angleArrived = true;
//      Serial.println(angleString);
//    }
//  }
//}
