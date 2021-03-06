#include "MessageParser.h"

MessageParser::MessageParser(Logging *_logger)
{
  logger = _logger;
}

bool MessageParser::deserialize(DynamicJsonDocument &doc, char *message)
{
  bool rc = true;

  DeserializationError err = deserializeJson(doc, message);
  logger->printlnLog("Memory usage : %d", doc.memoryUsage());
  if (err)
  {
    logger->printlnLog("parseObject() failed : %s", err.c_str());
    rc = false;
  }
  return rc;
}

void MessageParser::updateConfigurationResponse(CONFIG &conf, char *response)
{
  DynamicJsonDocument jsonBufferOut(GET_JSON_BUFFER);
  if (deserialize(jsonBufferOut, response))
  {
    if (checkRestError(jsonBufferOut) == REST_RET_OK)
    {
      JsonObject data = jsonBufferOut["data"];
      int needUpdate = data["needUpdate"];
      if (true) //needUpdate == 1)
      {
        JsonObject configuration = data["configuration"];
        conf.tempMeasure = configuration["tempMeasure"];
        conf.serverStatus = configuration["statusThermostat"];
        conf.timeZoneOffset = configuration["timeZoneOffset"];
        logger->printlnLog(
            "CheckUpdate : TempMeasure %d , statusThermostat %d , timeZoneOffset %d",
            conf.tempMeasure, conf.serverStatus, conf.timeZoneOffset);
        //TODO manage missing configuration
        JsonObject currentTempProgram =
            data["currentTempProgram"];

        conf.minTemp = currentTempProgram["minTemp"];
        conf.minTempManual = currentTempProgram["minTempManual"];
        JsonArray dayProgramming = currentTempProgram["dayProgramming"];
        int numDay = dayProgramming.size();
        for (int i = 0; i < numDay; i++)
        {
          JsonVariant dayP = dayProgramming[i];
          //JsonObject &dayP = _dayP;
          int idDay = dayP["idDay"];
          if (idDay >= 0 && idDay < MAX_DAY)
          {
            conf.day[idDay].day = idDay;
            JsonArray prog = dayP["prog"];
            int numProg = prog.size();
            if (numProg > MAX_PROGDAY)
              numProg = MAX_PROGDAY;
            conf.day[idDay].numProg = numProg;
            logger->printlnLog(
                "CheckUpdate : Day %d - DefTemp %f - NumProg %d",
                idDay, conf.minTemp,
                conf.day[idDay].numProg);
            //myprintln(printBuffer);
            for (int j = 0; j < numProg; j++)
            {
              JsonVariant proge = prog[j];

              //JsonObject &proge = _proge;
              conf.day[idDay].prog[j].minTemp =
                  proge["minTemp"];
              conf.day[idDay].prog[j].timeStart =
                  proge["timeStart"];
              conf.day[idDay].prog[j].timeEnd =
                  proge["timeEnd"];
              int pri = proge["priorityDisp"];
              if (pri == 0)
                pri = conf.key;
              conf.day[idDay].prog[j].priorityDisp = pri;
              logger->printlnLog(
                  "CheckUpdate : Prog %d - Temp %f - Start %d - End %d - Priority %d",
                  j, conf.day[idDay].prog[j].minTemp,
                  conf.day[idDay].prog[j].timeStart,
                  conf.day[idDay].prog[j].timeEnd,
                  conf.day[idDay].prog[j].priorityDisp);
            }
          }
        }
        conf.progLoaded = true;
        logger->printlnLog("CheckUpdate : Configuration updated");
      }
      else
      {
        logger->printlnLog("CheckUpdate : Configuration not updated");
      }
    }
  }
}

bool MessageParser::preparaMonitorDataRequest(CONFIG &config, SENSORDATA &sensorData, DynamicJsonDocument &jsonBuffer)
{
  bool send = false;
  if (sensorData.numItem == 0)
  {
    logger->printlnLog("Monitor... nothing to send");
  }
  else
  {
    jsonBuffer["macAddress"] = config.macAddress;
    jsonBuffer["temperature"] = sensorData.totalTemperature / sensorData.numItem;
    jsonBuffer["pressure"] = sensorData.totalPressure / sensorData.numItem / 100.0;
    jsonBuffer["light"] = sensorData.totalLight / sensorData.numItem;
    jsonBuffer["humidity"] = sensorData.humidity / sensorData.numItem;
    //jsonBuffer["statusThermostat"] = config.clientStatus;
    jsonBuffer["numSurveys"] = sensorData.numItem;
    send = true;
  }
  return send;
}

void MessageParser::preparaWiFiRegisterRequest(CONFIG &config, DynamicJsonDocument &jsonBuffer)
{
  jsonBuffer["flagLcd"] = config.flagLcd;
  jsonBuffer["flagLightSensor"] = config.flagLightSensor;
  jsonBuffer["flagMotionSensor"] = config.flagMotionSensor;
  jsonBuffer["flagTemperatureSensor"] = config.flagTemperatureSensor;
  jsonBuffer["flagPressureSensor"] = config.flagPressureSensor;
  jsonBuffer["flagHumiditySensor"] = config.flagHumiditySensor;
  jsonBuffer["flagReleTemp"] = config.flagReleTemp;
  jsonBuffer["flagReleLight"] = config.flagReleLight;
  jsonBuffer["macAddress"] = config.macAddress;
  jsonBuffer["ipAddress"] = config.ipAddress;
  jsonBuffer["deviceType"] = DEVICETYPE_ARDUINO;
}

int MessageParser::checkRestError(DynamicJsonDocument &doc)
{
  JsonObject error = doc["error"];
  int rc = error["code"];
  const char *message = error["message"];
  logger->printlnLog("Return code %d - %s", rc, message);
  return rc;
}
