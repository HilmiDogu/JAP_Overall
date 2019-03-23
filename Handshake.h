#ifndef HANDSHAKE_H
#define HANDSHAKE_H
#include <Arduino.h>
#define DEBUG_MODE    0
#define LED_RED       11
#define LED_GREEN     12
#define LED_BLUE      13
#define LED_YELLOW    14
enum CATCH_STATE{INIT, CATCHER, CATCHED};
enum HANDSHAKE_STATE{NORMAL, CATCH_REQUEST, CATCH_CONFIRM, CATCH_REJECT, CATCH_RESULT};

bool ESP_IfDataArrived();
bool Console_IfDataArrived();
void Both_WaitForResponse();
void ESP_WaitForResponse();
void Console_WaitForResponse();
String Console_ReadString();
String ESP_ReadString();
void Console_WriteString(String);
void ESP_WriteString(String);
void Console_WriteCharArray(const char *);
void ESP_WriteCharArray(const char *);
bool printATcommand(const char *);
String ConfigCWSAP();
String ConfigCWJAP();
String ConfigRemoteIP();
String ConfigStationIP();
void ListRemoteNetworks();
bool waitForReconnect();
void waitForConnection();
void WiFi_Init();
void WiFi_LED_Write(CATCH_STATE, HANDSHAKE_STATE);

#endif
