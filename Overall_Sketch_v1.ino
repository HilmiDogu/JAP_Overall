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

/*
 * Optimal PID equation related to elliptical track could be written 
 * by using tangent of the angle and multiplying with some constant.
 */
#include "Handshake.h"
#include "TB6612.h"
#include "Micro_Motor_Encoder.h"
#include "Sharp_5cm.h"
#include <PID_v1.h>

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
int lmpwmbase = 50;
int lmpwmadd  = 0;
int rmpwmbase = 50;
int rmpwmadd  = 0;
String angleString = "";    //  String to hold incoming data
int angleValue;
bool angleArrived = false;  //  whether the string is complete

double Input,Output,Setpoint, Error;
double Kp1=1, Ki1=0.25, Kd1=0.02;
int Outputmap1=0;
int Outputmap2=0;
PID myPID(&Input, &Output, &Setpoint, Kp1, Ki1, Kd1, DIRECT); 

void setup() {
    Serial.begin(115200);
    Sharp_5cm_Init();
    MicroMotorEncoderInit();
    WiFi_Init();
    Input = 90; 
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(125);
    Motor.Stop(2000);
}
void loop(){
   //State 0
       while(1){
         if(Current_State == NORMAL && Current_Catch_State == INIT){
           WiFi_LED_Write(INIT, NORMAL);
//          Motor Drive, Raspberry Pi communication and PID control here. (Maybe Distance Sensor Reading)
            if (angleArrived){
                Serial.println(angleString);
                if(!angleString.equals("NOP"))
                    angleValue = angleString.toInt();
                Setpoint = (double)angleValue;
                Error=Setpoint-Input;
            
                if(Error<50){
                    myPID.SetTunings(Kp1, Ki1, Kd1);
                }
                else if(Error>=50){
                    myPID.SetTunings(Kp2, Ki2, Kd2);
                }
                myPID.Compute();
             
                myPID.SetOutputLimits(-40,40);
            
                Motor.LeftMotorWrite(lmpwmbase - Output, FORWARD);
                Motor.RightMotorWrite(rmpwmbase + Output, FORWARD);
              
                angleString = "";
                angleArrived = false;
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
                continue;
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
                  continue;
              }
        }
      }
      else if (Current_Catch_State != CATCHER && Current_State == 1) {
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
          continue;
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
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the angleString:
    angleString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      angleArrived = true;
    }
  }
}
