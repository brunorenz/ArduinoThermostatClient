#ifndef HttpConnection_h
#define HttpConnection_h

#define HTTPGETBUFFER 1000

#include <WiFi101.h>
#include <ArduinoJson.h>
#include "WiFiConnection.h"
#include "TermClient.h"
#include "Logging.h"
#include <RTCZero.h>

class HttpConnection : public WiFiConnection
{
public:
  HttpConnection();
  bool initServerConnection(Client &client);
  bool waitServerResponse(Client &client);
  bool httpGetMethod(Client &client, char *getString);
  bool httpPostMethod(Client &client, char *postString, String &data);
  bool httpPostMethod(Client &client, char *postString, DynamicJsonDocument &doc);
  //void setRTC(RTCZero *rtc);
private:
  Logging logger;
  char httpBuffer[HTTPGETBUFFER];
};
#endif
