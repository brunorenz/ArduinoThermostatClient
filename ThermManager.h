#ifndef ThermManager_h
#define ThermManager_h

#include <RTCZero.h>
#include <ArduinoJson.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
//#include "HomeConnection.h"
#include "HttpConnection.h"
#include "Logging.h"

class ThermManager
{
public:
  ThermManager();
  void setHomeConnection(HttpConnection *hc, RTCZero *rtc);
  unsigned long wiFiRegister(CONFIG *conf);
  void checkUpdate(bool first, CONFIG *conf);
  void getCurrentData(TEMPDATA *outdata);
  void sendMonitorData(CONFIG *conf, SENSORDATA *sensorData);
  bool checkWiFiConnection();
  unsigned long getWiFiTime();

private:
  //
  void preparaWiFiRegisterRequest(CONFIG *config, JsonObject &root);
  unsigned long analizzaWiFiRegisterResponse(CONFIG *config, JsonObject *root);
  void analizzaCheckUpdateResponse(CONFIG *config, JsonObject &root);
  //void preparaCheckUpdateRequest(CONFIG *config, JsonObject& root);
  // internal operation
  void _checkUpdate(bool first, CONFIG *conf);
  void _getCurrentData(TEMPDATA *outdata);
  unsigned long _wiFiRegister(CONFIG *conf);
  void _wiFiRegisterWS(CONFIG *config);
  void _sendMonitorData(CONFIG *conf, SENSORDATA *sensorData);
  //
  unsigned long convertTime(double t);
  int checkRestError(DynamicJsonDocument &doc);
  bool checkThermConfiguration(CONFIG *conf);
  void formatDate(time_t t, char *buffer);

  //
  //HomeConnection *hc;
  HttpConnection *hc;
  RTCZero *rtc;
  Logging logger;
  //bool webSocketBegin;
  //
  WiFiClient client;
  String postData;
  //
  //int wifiRegisterCount;
  //int checkUpdateCount;
};
#endif
