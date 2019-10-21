#ifndef HomeConnection_h
#define HomeConnection_h

#define HTTPGETBUFFER 1000

#include <WiFi101.h>
#include <ArduinoJson.h>
#include "TermClient.h"
#include "Logging.h"
#include <RTCZero.h>

class HomeConnection
{
public:
  HomeConnection();
  bool checkConnection(bool wait = false);
  bool connect(bool wait = false);
  bool connect(char *_ssid, char *_pass, bool wait = false);
  void disconnect();
  int getConnectionStatus();
  unsigned long getTime();
  void getLocalIp(char *ip);
  void getMacAddress(char *mac);
  bool initServerConnection(WiFiClient *client);
  bool waitServerResponse(WiFiClient *client);
  bool httpGetMethod(WiFiClient *client, char *getString);
  bool httpPostMethod(WiFiClient *client, char *postString, String &data);
  void setRTC(RTCZero *rtc);
  //bool httpPostMethod(WiFiClient *client, char *postString, JsonObject& root);
private:
  bool reconnect(bool wait = false);
  Logging logger;
  RTCZero *rtc;
  char *ssid;
  char *pass;
  char httpBuffer[HTTPGETBUFFER];
};
#endif
