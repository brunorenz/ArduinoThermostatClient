#include "WiFiConnection.h"

WiFiConnection::WiFiConnection(Logging *_logger)
{
  logger = _logger;
  rtcUpdated = false;
}

void WiFiConnection::updateRTC(RTCZero &rtc, int timeZoneOffset)
{
  if (!rtcUpdated || timeZoneOffset != 0)
  {
    unsigned long now = getTime();
    if (now > 0)
    {
      now -= timeZoneOffset * 60;
      rtc.setEpoch(now);
      rtcUpdated = true;
    }
  }
}

bool WiFiConnection::connect(bool wait)
{
  // attempt to connect to WiFi network:
  //ssid = _ssid;
  //pass = _pass;
  int status = WiFi.status();
  logger->printlnLog(
      "Attempting to connect to WPA SSID: %s : Current WiFi status : %d",
      SECRET_SSID, status);
  if (status == WL_NO_SHIELD)
  {
    logger->printlnLog("WiFi shield not present");
    return false;
  }
  int count = 0;
  if (status != WL_CONNECTED)
  {
    while (status != WL_CONNECTED)
    {
      if (!wait && count > 0)
        break;

      //myprinlogger.printlnLog("Attempting to connect to WPA SSID: %s",ssid);
      //myprintln(ssid);

      // Connect to WPA/WPA2 network:
      status = WiFi.begin(SECRET_SSID, SECRET_PASS);

      // wait 10 seconds for connection:
      delay(CONNECT_WAIT_TIME);
      count++;
    }
    // Set Low Power Mode
    WiFi.lowPowerMode();
  }
  return status == WL_CONNECTED;
}

bool WiFiConnection::checkConnection(bool wait)
{
  return connect(wait);
}

bool WiFiConnection::reconnect(bool wait)
{
  disconnect();
  return connect(wait);
}

void WiFiConnection::disconnect()
{
  WiFi.disconnect();
}

unsigned long WiFiConnection::getTime()
{
  unsigned long epoch;
  int numberOfTries = 0, maxTries = 6;
  do
  {
    epoch = WiFi.getTime();
    numberOfTries++;
  } while ((epoch == 0) && (numberOfTries < maxTries));
  return epoch;
}

int WiFiConnection::getConnectionStatus()
{
  return WiFi.status();
}

void WiFiConnection::getLocalIp(char *lcdBuffer, int len)
{

  IPAddress ip = WiFi.localIP();
  snprintf(lcdBuffer, len, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
}

void WiFiConnection::getMacAddress(char *lcdBuffer,int len)
{
  byte mac[6];
  WiFi.macAddress(mac);
  logger->printlnLog("Mac Address : %s (%d)",lcdBuffer,sizeof(lcdBuffer));
  snprintf(lcdBuffer, len, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3],
           mac[2], mac[1], mac[0]);
}
