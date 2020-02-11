#ifndef WiFiConnection_h
#define WiFiConnection_h

#include "TermClient.h"
#include "Logging.h"
#ifdef ARDUINO_MKR1000
#include <WiFi101.h>
#include <RTCZero.h>
#else
//#include <WiFi101.h>
#include <ESP8266WiFi.h>
//#include <WiFi.h>
#endif


class WiFiConnection
{
public:
  WiFiConnection(Logging *_logger);
  bool checkConnection(bool wait = false);
  bool connect(bool wait = false);
  void disconnect();
  int getConnectionStatus();  
  void getLocalIp(char *ip, int len);
  void getMacAddress(char *mac, int len);
  #ifdef ARDUINO_MKR1000
  bool updateRTC(RTCZero &rtc, int timeZoneOffset = 0);
  unsigned long getTime();
  #endif
  bool initServerConnection(Client *client);

protected:
  bool reconnect(bool wait = false);
  bool rtcUpdated;
  Logging *logger;

private:
};
#endif
