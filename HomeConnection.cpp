#include "HomeConnection.h"

HomeConnection::HomeConnection()
{
  this->rtc = NULL;
}

void HomeConnection::setRTC(RTCZero *rtc)
{
  this->rtc = rtc;
  logger.setRTC(this->rtc);
}

bool HomeConnection::connect(char *_ssid, char *_pass, bool wait)
{
  // attempt to connect to WiFi network:
  ssid = _ssid;
  pass = _pass;
  int status = WiFi.status();
  logger.printlnLog(
      "Attempting to connect to WPA SSID: %s : Current WiFi status : %d",
      ssid, status);
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
      status = WiFi.begin(ssid, pass);

      // wait 10 seconds for connection:
      delay(CONNECT_WAIT_TIME);
      count++;
    }
    // Set Low Power Mode
    WiFi.lowPowerMode();
  }
  return status == WL_CONNECTED;
}

bool HomeConnection::checkConnection(bool wait)
{
  if (ssid == NULL)
    return false;
  return connect(ssid, pass, wait);
}

bool HomeConnection::reconnect(bool wait)
{
  disconnect();
  return connect(SECRET_SSID, SECRET_PASS, wait);
}

bool HomeConnection::connect(bool wait)
{
  return connect(SECRET_SSID, SECRET_PASS, wait);
}

void HomeConnection::disconnect()
{
  WiFi.disconnect();
}

unsigned long HomeConnection::getTime()
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

int HomeConnection::getConnectionStatus()
{
  return WiFi.status();
}

void HomeConnection::getLocalIp(char *lcdBuffer)
{

  IPAddress ip = WiFi.localIP();
  sprintf(lcdBuffer, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
}

void HomeConnection::getMacAddress(char *lcdBuffer)
{
  byte mac[6];
  WiFi.macAddress(mac);
  IPAddress ip = WiFi.localIP();
  sprintf(lcdBuffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3],
          mac[2], mac[1], mac[0]);
}

bool HomeConnection::httpGetMethod(WiFiClient *client, char *getString)
{
  if (initServerConnection(client))
  {
    sprintf(httpBuffer, "GET %s HTTP/1.1\r\nHost: %s:%d\r\n%s\r\n\r\n",
            getString,
            TERM_SERVER_URL, TERM_SERVER_PORT,
            HTTP_HEADGET);
    logger.printlnLog("HTTP CALL : %s", httpBuffer);
    client->println(httpBuffer);
    if (waitServerResponse(client))
    {
      return true;
    }
  }
  return false;
}
/**
bool HomeConnection::httpPostMethod(WiFiClient *client, char *postString, JsonObject& root) {

  if (initServerConnection(client)) {
    sprintf(httpBuffer, "POST %s HTTP/1.1\r\nHost: %s:%d\r\nContent-Length: %d\r\n%s\r\n\r\n",
            postString, TERM_SERVER_URL, TERM_SERVER_PORT, root.measureLength(), HTTP_HEADPOST);
    logger.printlnLog("HTTP CALL : %s(%d)", httpBuffer,strlen(httpBuffer));
    client->print(httpBuffer);
    WiFiClient client1;
    root.printTo(client1);
    if (waitServerResponse(client)) {
      return true;
    }
  }
  return false;
}
***/

// https://arduinojson.org/v6/doc/serialization/

bool HomeConnection::httpPostMethod(WiFiClient *client, char *postString, String &postData)
{

  if (initServerConnection(client))
  {
    sprintf(httpBuffer, "POST %s HTTP/1.1\r\nHost: %s:%d\r\nContent-Length: %d\r\n%s\r\n\r\n",
            postString, TERM_SERVER_URL, TERM_SERVER_PORT, postData.length(), HTTP_HEADPOST);
    logger.printlnLog("HTTP CALL : %s(%d)", httpBuffer, strlen(httpBuffer));
    client->print(httpBuffer);
    client->println(postData);
    if (waitServerResponse(client))
    {
      return true;
    }
  }
  return false;
}

bool HomeConnection::waitServerResponse(WiFiClient *client)
{
  bool rc = false;
  // Check HTTP status
  char status[32] = {0};
  client->readBytesUntil('\r', status, sizeof(status));
  if (strcmp(status, "HTTP/1.1 200 OK") != 0)
  {
    logger.printlnLog("Unexpected response: %s", status);
  }
  else
  {
    char endOfHeaders[] = "\r\n\r\n";
    client->setTimeout(GET_TIMEOUT);
    bool ok = client->find(endOfHeaders);
    if (ok)
    {
      logger.printlnLog("Received %d bytes..", client->available());
      return true;
    }
    else
      logger.printlnLog("No response or invalid response!");
  }
  return rc;
}

/*
  bool HomeConnection::waitServerResponse(WiFiClient *client) {
  logger.printlnLog("Received %d bytes..", client->available());
  return true;
  char endOfHeaders[] = "\r\n\r\n";
  client->setTimeout(GET_TIMEOUT);
  bool ok = client->find(endOfHeaders);
  if (ok) {
    logger.printlnLog("Received %d bytes..", client->available());
    return true;
  }
  logger.printlnLog("No response or invalid response!");
  return false;
  }
*/
bool HomeConnection::initServerConnection(WiFiClient *client)
{
  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  if (!client->connected())
    client->stop();
  bool rc = false;
  if (client->connected())
  {

    //while (client->available()) { client->read(); delay(10); }
    logger.printlnLog("Connection is available..");
    rc = true;
  }
  else
  {
    logger.printlnLog("Connecting to .. %s:%d ..", TERM_SERVER_URL,
                      TERM_SERVER_PORT);
    // if there's a successful connection:
    if (client->connect(TERM_SERVER_URL, TERM_SERVER_PORT))
    {
      logger.printlnLog("connected ..");
      rc = true;
    }
    else
    {
      logger.printlnLog("connection failed");
      //TODO
      // try to reconnect to WIFI
      reconnect();
    }
  }
  return rc;
}
