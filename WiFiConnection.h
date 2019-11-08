#ifndef WiFiConnection_h
#define WiFiConnection_h

//#define HTTPGETBUFFER 1000

#include <WiFi101.h>
#include <RTCZero.h>
#include "TermClient.h"
#include "Logging.h"

class WiFiConnection
{
public:
  WiFiConnection();
  bool checkConnection(bool wait = false);
  bool connect(bool wait = false);
  void disconnect();
  int getConnectionStatus();
  unsigned long getTime();
  void getLocalIp(char *ip);
  void getMacAddress(char *mac);
  //void setRTC(RTCZero *rtc);
  void updateRTC(RTCZero &rtc, int timeZoneOffset);
  bool initServerConnection(Client *client);

protected:
  bool reconnect(bool wait = false);
  //bool connect(char *_ssid, char *_pass, bool wait = false);
  Logging logger;

private:
  //RTCZero *rtc;
  //char *ssid;
  //char *pass;
  //char httpBuffer[HTTPGETBUFFER];
};
#endif
