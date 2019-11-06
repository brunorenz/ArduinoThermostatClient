#include "WiFiConnection.h"

WiFiConnection::WiFiConnection()
{
  this->rtc = NULL;
}

void WiFiConnection::setRTC(RTCZero *rtc)
{
  this->rtc = rtc;
  logger.setRTC(this->rtc);
}

bool WiFiConnection::connect(bool wait)
{
  // attempt to connect to WiFi network:
  //ssid = _ssid;
  //pass = _pass;
  int status = WiFi.status();
  logger.printlnLog(
      "Attempting to connect to WPA SSID: %s : Current WiFi status : %d",
      SECRET_SSID, status);
  if (status == WL_NO_SHIELD)
  {
    logger.printlnLog("WiFi shield not present");
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
  char temp[80];
  unsigned long epoch;
  int numberOfTries = 0, maxTries = 6;
  do
  {
    epoch = WiFi.getTime();
    numberOfTries++;
  } while ((epoch == 0) && (numberOfTries < maxTries));
  if (epoch == 0)
  {
    logger.printlnLog(temp, "Server NTP unreachable!!");
  }
  return epoch;
}

int WiFiConnection::getConnectionStatus()
{
  return WiFi.status();
}

void WiFiConnection::getLocalIp(char *lcdBuffer)
{

  IPAddress ip = WiFi.localIP();
  sprintf(lcdBuffer, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
}

void WiFiConnection::getMacAddress(char *lcdBuffer)
{
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(lcdBuffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3],
          mac[2], mac[1], mac[0]);
}
