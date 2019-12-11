#ifndef WiFiConnection_h
#define WiFiConnection_h

#include <WiFi101.h>
#include <RTCZero.h>
#include "TermClient.h"
#include "Logging.h"

class WiFiConnection
{
public:
  WiFiConnection(Logging *_logger);
  bool checkConnection(bool wait = false);
  bool connect(bool wait = false);
  void disconnect();
  int getConnectionStatus();
  unsigned long getTime();
  void getLocalIp(char *ip, int len);
  void getMacAddress(char *mac, int len);
  void updateRTC(RTCZero &rtc, int timeZoneOffset);
  bool initServerConnection(Client *client);

protected:
  bool reconnect(bool wait = false);
  bool rtcUpdated;
  Logging *logger;

private:
};
#endif
