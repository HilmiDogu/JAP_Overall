/*
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
#include "Handshake.h"
#include "TB6612.h"
#include "Micro_Motor_Encoder.h"
#define SHARP_5CM_FRONT 21
#define SHARP_5CM_FRONT_INT_NUMBER  2
static HANDSHAKE_STATE Current_State = NORMAL;
static CATCH_STATE Current_Catch_State = INIT;
static TB6612 Motor(STANDBY, RIGHT_PWM, LEFT_PWM, RIGHT_IN_1, RIGHT_IN_2, LEFT_IN_1, LEFT_IN_2);
extern String rcv;
extern String remoteID;
extern char rmtID[2];
extern long leftTickCount;
extern long rightTickCount;
long errorArray[20];
int errorIndex = 0;
long sumError = 0;
long preverror, currerror, differror, sumerror, errorarray[10];
int lmpwmbase = 140;
int lmpwmadd  = 0;
int rmpwmbase = 140;
int rmpwmadd  = 0;
void setup() {
  Serial.begin(115200);
  pinMode(SHARP_5CM_FRONT, INPUT);
  attachInterrupt(SHARP_5CM_FRONT_INT_NUMBER, Approach_5cm_Event, CHANGE);
  MicroMotorEncoderInit();
  //WiFi_Init();
//  Motor.Stop(1000);
}
void loop() {
  Motor.LeftMotorWrite(lmpwmbase - rmpwmadd, FORWARD);
  Motor.RightMotorWrite(rmpwmbase + rmpwmadd, FORWARD);
  delay(100);
   currerror = leftTickCount - rightTickCount;//sag motor daha hizli donuyor
   errorArray[errorIndex] = currerror;
   for(int i = 0; i < 20; i++) sumError += errorArray[i];
   errorIndex = (errorIndex + 1) % 20;
   
   differror = currerror - preverror;
   rmpwmadd = (int)((float)currerror*(-0.9))+ (int)((float)0*differror*(-0.1)) + (int)((float)0*sumError*(-0.005));
   preverror = currerror;
   if((rmpwmbase + rmpwmadd) > 255)rmpwmadd = 255;
   else if((rmpwmbase + rmpwmadd) < 0) rmpwmadd = 0;
//   if(error > 0){
//       rmpwmadd = (int)((float)error*(0.5));
//       if(rmpwmadd > 205)rmpwmadd = 205;
//       lmpwmadd = 0;
//   }
//   else {
//    lmpwmadd = (int)((float)error*(-0.5));
//        if(lmpwmadd > 205)lmpwmadd = 205;
//       rmpwmadd = 0; 
//    
//        
//   }
//   Serial.println(lmpwmadd);
//   Serial.println(rmpwmadd);
//  Serial.print(currerror);Serial.print(" ");Serial.println(differror);
  
  //Serial.println(leftTickCount);
  rightTickCount = 0;
  leftTickCount = 0;
  //delay(20);
  //  for (int i = 20; i <= 80; i = i + 10) {
  //    Motor.LeftMotorWrite(i, FORWARD);
  //    Motor.RightMotorWrite(i, FORWARD);
  //    delay(20);
  //  }
  //  Motor.LeftMotorWrite(80, FORWARD);
  //  Motor.RightMotorWrite(80, FORWARD);
  //  delay(100);
  //  for (int i = 80; i >= 20; i = i - 10) {
  //    Motor.LeftMotorWrite(i, FORWARD);
  //    Motor.RightMotorWrite(i, FORWARD);
  //    delay(20);
  //  }
  //  for (int i = 20; i <= 80; i = i + 10) {
  //    Motor.LeftMotorWrite(i, BACKWARD);
  //    Motor.RightMotorWrite(i, BACKWARD);
  //    delay(20);
  //  }
  //  Motor.LeftMotorWrite(80, BACKWARD);
  //  Motor.RightMotorWrite(80, BACKWARD);
  //  delay(100);
  //  for (int i = 80; i >= 20; i = i - 10) {
  //    Motor.LeftMotorWrite(i, BACKWARD);
  //    Motor.RightMotorWrite(i, BACKWARD);
  //    delay(20);
  //  }
//  Serial.print("Right diff: ");
//  Serial.println(rightDiffTime);
//  Serial.print("Left diff: ");
//  Serial.println(leftDiffTime);
  //State 0
  //    while(1){
  //      if(Current_State == NORMAL){
  //        if(DEBUG_MODE) Console_WriteCharArray("Type one character to catch, or wait for being catched");
  //        WiFi_LED_Write(INIT, NORMAL);
  //
  //        //Motor Drive, Raspberry Pi communication and PID control here. (Maybe Distance Sensor Reading)
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //        Both_WaitForResponse();
  //      }
  //      if (Console_IfDataArrived() && Current_State == NORMAL) {            //If character is came from console, it catched robot from back
  //        Serial.read();                                                //Clear receive buffer
  //        ESP_WriteCharArray("AT+CIPSEND=0,4");
  //        ESP_WaitForResponse();
  //        rcv = Serial1.readString();
  //        if (rcv.indexOf((char*)"OK")) ESP_WriteCharArray("1000");
  //        if(DEBUG_MODE) Console_WriteCharArray("Catch request(1000) is sent.");
  //        WiFi_LED_Write(CATCHER, CATCH_REQUEST);
  //        Current_Catch_State = CATCHER;
  //        Current_State = CATCH_REQUEST;
  //
  //      }
  //      else if (ESP_IfDataArrived() && Current_State == NORMAL) {                     //If character is came from ESP8266, it is catched by another robot
  //        String s = Serial1.readString();
  //        String ss = s.substring(13);
  //        String ID = s.substring(11,13);
  //        if(DEBUG_MODE){
  //          Serial.println(s);
  //          Serial.println(ss);
  //          Serial.println(ID);
  //          Serial.println(ID.equals(remoteID));
  //        }
  //        if (ss.indexOf("00") >= 0 && ID.equals(remoteID)) if(DEBUG_MODE) Console_WriteCharArray("Catch request(0400) is received.");
  //        else if(!ID.equals(remoteID)){
  //          if(DEBUG_MODE) Console_WriteCharArray("Wrong ID.");
  //          continue;
  //        }
  //        Current_Catch_State = CATCHED;
  //        Current_State = CATCH_REQUEST;
  //      }
  //      //State 1
  //      if (Current_Catch_State == CATCHER && Current_State == CATCH_REQUEST) {
  //        if (ESP_IfDataArrived()) {
  //          String s = Serial1.readString();
  //          String ss = s.substring(13);
  //          String ID = s.substring(11,13);
  //          if(DEBUG_MODE){
  //            Serial.println(s);
  //            Serial.println(ss);
  //            Serial.println(ID);
  //            Serial.println(ID.equals(remoteID));
  //          }
  //          if (ss.indexOf("01") >= 0 && ID.equals(remoteID)) {
  //            if(DEBUG_MODE) Console_WriteCharArray("Catch is confirmed(0401).");
  //            WiFi_LED_Write(CATCHER, CATCH_CONFIRM);
  //            Current_State = CATCH_CONFIRM;
  //          }
  //          else if (ss.indexOf("11") >= 0 && ID.equals(remoteID)) {
  //            if(DEBUG_MODE) Console_WriteCharArray("Catch is not confirmed(0411).");
  //            WiFi_LED_Write(CATCHER, CATCH_REJECT);
  //            delay(1000);
  //            Current_Catch_State = INIT;
  //            Current_State = NORMAL;
  //            continue;
  //          }
  //
  //        }
  //      }
  //      else if (!Current_Catch_State && Current_State == 1) {
  //        if(DEBUG_MODE) Console_WriteCharArray("Press '1' for catched, '2' for not catched.");
  //        while (!Serial.available());
  //        char ch = Serial.read();
  //        if (ch == '1') {
  //          if(DEBUG_MODE) Console_WriteCharArray("Confirm for catch(1001) sent.");
  //          ESP_WriteCharArray("AT+CIPSEND=0,4");
  //          ESP_WaitForResponse();
  //          rcv = Serial1.readString();
  //          if (rcv.indexOf((char*)"OK")) ESP_WriteCharArray("1001");
  //          Current_State = CATCH_RESULT;
  //        }
  //        else if (ch == '2') {
  //          if(DEBUG_MODE) Console_WriteCharArray("No confirm for catch(1011) sent.");
  //          ESP_WriteCharArray("AT+CIPSEND=0,4");
  //          ESP_WaitForResponse();
  //          rcv = Serial1.readString();
  //          if (rcv.indexOf((char*)"OK")) ESP_WriteCharArray("1011");
  //          ESP_WaitForResponse();
  //          rcv = Serial1.readString();
  //          Current_State = NORMAL;
  //          Current_Catch_State = INIT;
  //          continue;
  //        }
  //      }
  //        //State 2
  //        else if(Current_Catch_State == CATCHER && Current_State == CATCH_RESULT){
  //            if(DEBUG_MODE) Console_WriteCharArray("Stop command(1010) sent.");
  //            ESP_WriteCharArray("AT+CIPSEND=0,4");
  //            ESP_WaitForResponse();
  //            rcv = Serial1.readString();
  //            if (rcv.indexOf((char*)"OK")) ESP_WriteCharArray("1010");
  //            ESP_WaitForResponse();
  //            rcv = Serial1.readString();
  //            WiFi_LED_Write(CATCHER, CATCH_RESULT);
  //            delay(10000);
  //            WiFi_LED_Write(INIT, NORMAL);
  //            while(1);//End of the handshake
  //        }
  //        else if(Current_Catch_State == CATCHED && Current_State == CATCH_RESULT){
  //            ESP_WaitForResponse();
  //            Serial1.readString();
  //            ESP_WaitForResponse();
  //            String s = Serial1.readString();
  //            String ss = s.substring(13);
  //            String ID = s.substring(11,13);
  //            if(DEBUG_MODE){
  //              Serial.println(s);
  //              Serial.println(ss);
  //              Serial.println(ID);
  //              Serial.println(ID.equals(remoteID));
  //            }
  //            if (ss.indexOf("10") >= 0 && ID.equals(remoteID)) if(DEBUG_MODE) Console_WriteCharArray("Stop command(0410) is received.");
  //            WiFi_LED_Write(CATCHER, CATCH_RESULT);
  //            delay(10000);
  //            WiFi_LED_Write(INIT, NORMAL);
  //            while(1);//End of the handshake
  //        }
  //
  //  }
}
void Approach_5cm_Event(){
    if(!digitalRead(SHARP_5CM_FRONT)){
        Motor.Stop(0);   
        while(!digitalRead(SHARP_5CM_FRONT));
        Motor.Resume();    
    }    

}
