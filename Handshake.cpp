#include "Handshake.h"
#include <Arduino.h>
String rcv;
String remoteID;
char rmtID[2];

bool ESP_IfDataArrived(){
  return Serial1.available();  
}
bool Console_IfDataArrived(){
  return Serial.available();
}
void ESP_WaitForResponse(){
  while (!Serial1.available());
}
void Console_WaitForResponse(){
  while (!Serial.available());
}
void Both_WaitForResponse(){
  while(!Serial.available() && !Serial1.available());
}
String Console_ReadString(){
  return Serial.readString();
}
String ESP_ReadString(){
  return Serial1.readString();
}
void Console_WriteString(String str){
  Serial.println(str);
}
void Console_WriteCharArray(const char *str){
  Serial.println(str);  
}
void ESP_WriteCharArray(const char *str){
  Serial1.println(str);  
}
void ESP_WriteString(String str){
  Serial1.println(str);
}
bool printATcommand(const char *str) {
  ESP_WriteCharArray(str);
  String response;
  int trial = 0;
  while (1) {   //if not found indexOf returns -1
    ESP_WaitForResponse();
    response = ESP_ReadString();
    if (DEBUG_MODE) {
      Console_WriteString(response);
      Serial.println(trial);
    }
    if (response.indexOf((char*)"ERROR") >= 0)trial++;
    if (response.indexOf((char*)"OK") >= 0)return 1;
    if (trial >= 5) return 0;
  }
}
String ConfigCWSAP() {
  String temp, ssid, password, channel;
  Console_WriteCharArray("SSID: ");
  Console_WaitForResponse();//Console_WaitForResponse();
  ssid = Console_ReadString();
  Console_WriteCharArray("\nPassword: ");
  Console_WaitForResponse();
  password = Console_ReadString();
  Console_WriteCharArray("\nChannel: ");
  Console_WaitForResponse();
  channel = Console_ReadString();
  Serial.println();
  temp = "AT+CWSAP=\"" + ssid + "\",\"" + password + "\"," + channel + ",3\r\n";
  return temp;
}
String ConfigCWJAP() {
  String temp, ssid, password;
  Console_WriteCharArray("Remote device SSID(\"#*\" for rescan): ");
  Console_WaitForResponse();
  ssid = Console_ReadString();
  if(ssid.indexOf("#*") >= 0)return ssid;
  Console_WriteCharArray("\nPassword: ");
  Console_WaitForResponse();
  password = Console_ReadString();
  Serial.println();
  temp = "AT+CWJAP=\"" + ssid + "\",\"" + password + "\"\r\n";
  return temp;
}
String ConfigRemoteIP() {
  String temp, ip;
  char wrt[60];
  Console_WriteCharArray("Remote IP: ");

  Console_WaitForResponse();
  ip = Console_ReadString();
  temp = "AT+CIPSTART=0,\"TCP\",\"" + ip + "\",5000\r\n";
  return temp;
}
String ConfigStationIP(){
  String temp, staip;
  Console_WriteCharArray("Station IP:");
  Console_WaitForResponse();
  staip = Console_ReadString();
  temp = "AT+CIPSTA=\"" + staip + "\"\r\n";
  return temp;
}
void ListRemoteNetworks() {
  Serial1.println("AT+CWLAP\r\n");
  delay(1000);
  String response;
  char resp[] = "OK";
  while (response.indexOf(resp) < 0) {
    ESP_WaitForResponse();
    response = ESP_ReadString();
    if(response.indexOf("CWLAP") >= 0)
      ESP_WriteString(response);
  }
}
bool waitForReconnect(){
  Console_WriteCharArray("Wating for previous connection established.");
  while (1) {
    if(Serial.available()) return 0;
      if(Serial1.available()){
      String response = ESP_ReadString();
      if (DEBUG_MODE) ESP_WriteString(response);
      if (response.indexOf("GOT IP") >= 0) {
        Console_WriteCharArray("Connection found.");
        return 1;
      }
      else if(response.indexOf("FAIL") >=0){
        Console_WriteCharArray("Connection failed!");
        return 0;
      }
    }
  }
}
void waitForConnection() {
  Console_WriteCharArray("Waiting for a client to connect.");
  while (1) {
    ESP_WaitForResponse();
    String response = ESP_ReadString();
    if (DEBUG_MODE) ESP_WriteString(response);
    char resp[] = "CONNECT";
    if (response.indexOf(resp) >= 0) {
      Console_WriteCharArray("Connection started");
      return;
    }
  }
}
void WiFi_Init(){
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  
  Console_WriteCharArray("Press 'S' for Server, 'C' for Client: ");
  Console_WaitForResponse();
  char choose = Serial.read();
  Console_WriteCharArray("Remote ID:");
  Console_WaitForResponse();
  remoteID = Console_ReadString();
  rmtID[0] = remoteID[0]; rmtID[1] = remoteID[1];
  if(DEBUG_MODE) Serial.println(rmtID);
  if(!DEBUG_MODE) printATcommand("ATE0");
  while (1) {
	if (choose == 'S' || choose == 's') {             //If server is selected
	  if (printATcommand("AT"))                        Console_WriteCharArray("\nCommunication with ESP8266:OK");
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  if (printATcommand("AT+RST"))                    Console_WriteCharArray("ESP8266 Reset.");
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  if (printATcommand("AT+CIPMUX=1"));
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  if (printATcommand("AT+CWMODE=2"))               Console_WriteCharArray("Device set as Access Point.");
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  char wrt[60];
	  String cwxap = ConfigCWSAP();
	  cwxap.toCharArray(wrt, 60);
	  if (printATcommand(wrt))                         Console_WriteCharArray("Server SSID and password is configured.");
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  if (printATcommand("AT+CIPAP=\"192.168.1.10\"")) Console_WriteCharArray("Server IP is configured as 192.168.1.10.");
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  if (printATcommand("AT+CIPSERVER=1,5000"))       Console_WriteCharArray("Server is created and port number is set as 5000.");
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  if (printATcommand("AT+CIPSTO=7000"))            Console_WriteCharArray("Server timeout is configured.");
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  waitForConnection();
	  break;
	}
	else if (choose == 'C' || choose == 'c') {        //if client is selected
	  if (printATcommand("AT"))                        Console_WriteCharArray("\nCommunication with ESP8266: OK");
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  if (printATcommand("AT+RST"))                    Console_WriteCharArray("ESP8266 Reset.");
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  if (printATcommand("AT+CIPMUX=1"));
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  if (printATcommand("AT+CWMODE=1"))               Console_WriteCharArray("Device set as station.");
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  if (!printATcommand("AT+CWLAPOPT=1,2"))          Console_WriteCharArray("ERROR"); 
	  if (!printATcommand("AT+CWDHCP=1,1"))            Console_WriteCharArray("ERROR"); 
	  String cwjap; 
	  char wrt[60];
	  while(1){
		ListRemoteNetworks();
		cwjap = ConfigCWJAP();
		if(cwjap != "#*"){
		  break;
		}
		Console_WriteCharArray("\nRescanning."); 
	  }
	  cwjap.toCharArray(wrt, 60);
	  if (printATcommand(wrt))                        Console_WriteCharArray("SSID and password of remote device is configured.");
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  if (printATcommand("AT+CIPSTATUS"));
	  else if(DEBUG_MODE) Console_WriteCharArray("ERROR");
	  String cipstart = ConfigRemoteIP();
	  cipstart.toCharArray(wrt, 60);
	  if (printATcommand(wrt))                        Console_WriteCharArray("Remote connection established!");
	  else                                            Console_WriteCharArray("Error when establishing remote connection!");
	  break;
	}
  }
}
void WiFi_LED_Write(CATCH_STATE cat, HANDSHAKE_STATE state){
  if(cat == CATCHER){
    switch(state){
      case NORMAL:{
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_YELLOW, LOW);
        break;  
      }
      case CATCH_REQUEST:{
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_YELLOW, LOW);
        break;  
      }
      case CATCH_CONFIRM:{
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_YELLOW, LOW);        
      }
      case CATCH_REJECT:{
          digitalWrite(LED_RED, LOW);
          digitalWrite(LED_BLUE, LOW);
          digitalWrite(LED_GREEN, LOW);
          digitalWrite(LED_YELLOW, HIGH);
      }
      case CATCH_RESULT:{
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_YELLOW, LOW);
        break;  
      }
    }  
  }
  else if(cat == CATCHED){
    switch(state){
      case NORMAL:{
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_YELLOW, LOW);
        break;  
      }
      case CATCH_REQUEST:{
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_YELLOW, LOW);
        break;  
      }
      case CATCH_RESULT:{
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_YELLOW, LOW);
        break;  
      }
    }  
  }
  else if(cat == INIT){
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
  } 
}
