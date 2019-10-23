#ifndef HttpConnection_h
#define HttpConnection_h

#define HTTPGETBUFFER 200
#define ARDUINOJSON_USE_LONG_LONG 1 

#include <WiFi101.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>
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
  bool httpPostMethod(Client &client, char *postString, DynamicJsonDocument &doc);

  bool deserializeJsonResponse(Client &client, DynamicJsonDocument &doc);

private:
  char httpBuffer[HTTPGETBUFFER];
};
#endif
