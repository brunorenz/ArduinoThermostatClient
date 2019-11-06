#ifndef MessageParser_h
#define MessageParser_h

#define ARDUINOJSON_USE_LONG_LONG 1 

#include <RTCZero.h>
#include <ArduinoJson.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include "Logging.h"

class MessageParser
{
public:
  MessageParser();
  void preparaWiFiRegisterRequest(CONFIG *config, JsonObject &root);

private:
  int checkRestError(DynamicJsonDocument &doc);
  RTCZero *rtc;
  Logging logger;
};
#endif
