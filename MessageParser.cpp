#include "MessageParser.h"

MessageParser::MessageParser()
{
  //webSocketBegin = false;
  //wifiRegisterCount = 0;
  //checkUpdateCount = 0;
  //postData.reserve(2000);
}

void MessageParser::preparaWiFiRegisterRequest(CONFIG *config, DynamicJsonDocument &root)
{
  DynamicJsonDocument jsonBuffer(GET_JSON_BUFFER);
  char macAddress[50];
  char ipAddress[50];
  // get IP and MAC address
  hc->getMacAddress(macAddress);
  hc->getLocalIp(ipAddress);

  jsonBuffer["flagLcd"] = config->flagLcd;
  jsonBuffer["flagLightSensor"] = config->flagLightSensor;
  jsonBuffer["flagMotionSensor"] = config->flagMotionSensor;
  jsonBuffer["flagReleTemp"] = config->flagReleTemp;
  jsonBuffer["flagReleLight"] = config->flagReleLight;
  jsonBuffer["macAddress"] = macAddress;
  jsonBuffer["ipAddress"] = ipAddress;
}

int MessageParser::checkRestError(DynamicJsonDocument &doc)
{
  JsonObject error = doc["error"];
  int rc = error["code"];
  const char *message = error["message"];
  logger.printlnLog("Return code %d - %s", rc, message);
  return rc;
}
