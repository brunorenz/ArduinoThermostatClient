/*
  TermClient main module
*/

//#include <avr/io.h>
//#include <avr/wdt.h>
#include <Adafruit_SleepyDog.h>
#include "TermClient.h"
#include "HttpConnection.h"
#include "ThermManager.h"
#include "MessageParser.h"
#include "MemoryFree.h"
#include "Logging.h"

#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <RTCZero.h>
#include <time.h>
#include <string.h>
#include <ArduinoJson.h>
#include <ArduinoMqttClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>

int status = WL_IDLE_STATUS; // the WiFi radio's status
boolean LCD = false;
boolean BMP280 = false;
char temp[250];

Logging logger;
RTCZero rtc;

WiFiConnection wifi;
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
MessageParser messageParser;

ThermManager tm;
#ifdef USE_MQ

#else
HttpConnection hc;
#endif

// LCD
LiquidCrystal_I2C lcd(ADDRESS_LCD, 20, 4);

// Temperature Sensor
#ifdef BMP
Adafruit_BMP280 bme;
#endif
#ifdef BME
Adafruit_BME280 bme;
#endif

bool checkI2CAddress(int address);
void readTemperature(boolean init);
float getManagedTemperature(int pryDisp, CONFIG *conf);
bool checkConfiguration(CONFIG *conf, bool connectionAvailable);
bool checkThermostatStatus(float cT, CONFIG *conf, boolean connectionAvailable);
void loopREST();

long timeoutCallMonitor;
long timeoutReadTemperature;
long timeoutSetTemperatureRele;
long timeoutCheckConfiguration;
CONFIG config;
SENSORDATA sensorData;

const int sensorPin = A1;
const int relayPin = 9;
int count = 0;
bool willSent = 0;

// GEstione LCD
uint8_t bell[8] = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};

/**
 * Reset function
 */

void (*resetFunc)(void) = 0; //declare reset function at address 0

int checkServerConnection(WiFiClient &client)
{
  uint8_t connected = client.connected();
}
/**
  Setup .. initialization
*/
void setup()
{

  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  // wait serial to start
  delay(2000);

  // initialize configuration data
  config.progLoaded = false;
  config.serverStatus = STATUS_OFF;
  config.clientStatus = STATUS_OFF;
  config.key = 0;
  config.flagLcd = 0;
  config.flagLightSensor = 0;
  config.flagMotionSensor = 0;
  config.flagReleTemp = 0;
  config.flagReleLight = 0;
  config.registered = false;
  for (int i = 0; i < MAX_DAY; i++)
  {
    config.day[i].day = -1;
    config.day[i].numProg = 0;
  }
  config.macAddress[0] = 0;
  config.ipAddress[0] = 0;

#ifdef FLAGLIGHT
  config.flagLightSensor = 1;
#endif
#ifdef FLAGMOTION
  config.flagMotionSensor = 1;
#endif
#ifdef FLAGRELEMOTION
  config.flagReleLight = 1;
#endif
#ifdef FLAGRELETEMP
  config.flagReleTemp = 1;
#endif

  // initialize I2C comunication
  Wire.begin();
  // Initialize rtc
  rtc.begin();
  LCD = checkI2CAddress(ADDRESS_LCD);
  BMP280 = checkI2CAddress(ADDRESS_BMP280);
  if (LCD)
  {
    lcd.init();      //initialize the lcd
    lcd.backlight(); //open the backlight
    config.flagLcd = 1;
  }
  if (BMP280)
  {
    bme.begin();
  }
  logger.setRTC(&rtc);
  logger.printlnLog("ThermostatClient start...");
  logger.printlnLog("LCD status    .. %s", LCD ? "OK" : "KO");
  logger.printlnLog("BMP280 status .. %s", BMP280 ? "OK" : "KO");

  pinMode(LED_BUILTIN, OUTPUT);
  // SET Pin for RELE
  pinMode(relayPin, OUTPUT);
  // Set resolution for analog read
  analogReadResolution(10);
  setupMQ();
  // Set connection to Thermostat Manager
  //wifi.setRTC(&rtc);
  //tm.setHomeConnection(&hc, &rtc); //, &client);
  timeoutCallMonitor = millis();
  timeoutReadTemperature = timeoutCallMonitor;
  // call as soon as possible the first time
  timeoutCheckConfiguration = 0;
  timeoutSetTemperatureRele = 0;
  // read initial temperature values
  readTemperature(true);
  // imposto Watchdog Timer a 8 Secondi
  //wdt_enable(WDTO_8S);
  Watchdog.enable(8000);
}

void setupMQ()
{
  if (checkWIFIConnection())
    checkMQConnection();
}

void setupREST()
{
  // Set connection to Thermostat Manager
  // wifi.setRTC(&rtc);
  //tm.setHomeConnection(&hc, &rtc); //, &client);
}

void loop()
{
#ifdef WEBSOCKET
  loopWS();
#else
  loopMQ();
#endif
}
/**
 * Check and Manage WiFi connection
 */
bool checkWIFIConnection()
{
  bool rc = wifi.connect();
  if (rc)
  {
    // get IP and MAC address
    if (strlen(config.macAddress) == 0)
      wifi.getMacAddress(config.macAddress);
    if (strlen(config.ipAddress) == 0)
      wifi.getLocalIp(config.ipAddress);
  }
  return rc;
}
/**
 * Check and Manage MQTT Connection 
 */
bool checkMQConnection()
{
  int rc = 1;
  if (!mqttClient.connected())
  {
    // set a will message, used by the broker when the connection dies unexpectantly
    // you must know the size of the message before hand, and it must be set before connecting
    sendWillMessage();
    const char broker[] = TERM_SERVER_MQ;
    int port = 1883;
    rc = mqttClient.connect(broker, port);
    if (!rc)
    {
      Serial.print("MQTT connection failed! Error code = ");
      Serial.println(mqttClient.connectError());
    }
    else
    {
      // register listener
      mqttClient.onMessage(onMqttMessage);
      // subscribe topic
      int subscribeQos = 1;
      const char topic1[] = TOPIC_UPDATEPROG;
      mqttClient.subscribe(topic1, subscribeQos);      
      const char topic2[] = TOPIC_UPDATETEMP;
      mqttClient.subscribe(topic2, subscribeQos);
    }
  }
  return rc == 1;
}

void loopMQ()
{
  long now = millis();
  bool checkTemperature = (now - timeoutReadTemperature) > WAIT_READ_TEMPERATURE;

  bool wifiConnectionAvailable = checkWIFIConnection();
  if (wifiConnectionAvailable)
  {
    bool mqttConnectionAvailable = checkMQConnection();
    if (mqttConnectionAvailable)
    {
      logger.printlnLog("Connection WIFI and MQ ok!");
      if (!config.registered)
      {
        sendWiFiRegisterMessage(config);
      }
    }
  }
  // read sensor data
  if (checkTemperature)
  {
    readTemperature(false);
    timeoutReadTemperature = now;
  }
  logger.printlnLog("Check FreeMemory %d", freeMemory());
  mqttClient.poll();
  delay(WAIT_MAIN_LOOP);
  // Watchdog Timer
  Watchdog.reset();

  // check WiFi Connection
  // if true check MQ Connection
  // if true WifI WiFiRegister
  // if (config not found  getConnection)
  // subscribe (updatePrograming and updateThemperature)
}

void sendWillMessage()
{
  // set a will message, used by the broker when the connection dies unexpectantly
  // you must know the size of the message before hand, and it must be set before connecting
  char willPayload[100];
  sprintf(willPayload, "{\"macAddress\":\"%s\"}", config.macAddress);
  bool willRetain = true;
  int willQos = 1;
  char willTopic[] = TOPIC_LASTWILL;

  mqttClient.beginWill(willTopic, strlen(willPayload), willRetain, willQos);
  mqttClient.print(willPayload);
  mqttClient.endWill();
  logger.printlnLog("Send Will message %s", willPayload);
  willSent = 1;
}

void sendWiFiRegisterMessage(CONFIG &cfg)
{
  DynamicJsonDocument jsonBuffer(GET_JSON_BUFFER);
  messageParser.preparaWiFiRegisterRequest(cfg, jsonBuffer);
  int jsonMessageLen = measureJson(jsonBuffer);
  char jsonMessage[jsonMessageLen+1];
  serializeJson(jsonBuffer, jsonMessage, sizeof(jsonMessage));
  logger.printlnLog("Send WiFiRegister message %s (%d - %d)", jsonMessage,jsonMessageLen,strlen(jsonMessage));
  bool retained = false;
  int qos = 1;
  bool dup = false;
  char outTopic[] = TOPIC_WIFI;
  
  mqttClient.beginMessage(outTopic, strlen(jsonMessage), retained, qos, dup);
  mqttClient.print(jsonMessage);
  mqttClient.endMessage();
  cfg.registered = 1;
  logger.printlnLog("Send WiFiRegister done!");
}

void loopREST()
{
  // check connection
  // if (connect available)
  //    check configuration
  //    if configuration available
  //      normal management
  // else
  //    if configuration available
  //      only check temperature according last configuration available

  // recheck Serial
  //available = Serial;
  //
  bool wifiConnectionAvailable = tm.checkWiFiConnection();
  bool configurationAvailable = checkConfiguration(&config, wifiConnectionAvailable);
  long now = millis();
  bool checkTemperature = (now - timeoutReadTemperature) > WAIT_READ_TEMPERATURE;
  bool checkConfiguration = (now - timeoutCheckConfiguration) > WAIT_CALL_CHECK;
  bool checkReleTemperature = (now - timeoutSetTemperatureRele) > WAIT_SETRELE_TEMPERATURE;
  bool sendMonitorData = (now - timeoutCallMonitor) > WAIT_CALL_MONITOR;

  /*
    logger.printlnLog("wifi : %d, configAvailable : %d, checkTemperature : %d, checkConfiguration : %d, checkReleTemperature : %d, sendMonitorData :%d",
                    wifiConnectionAvailable, configurationAvailable, checkTemperature, checkConfiguration, checkReleTemperature, sendMonitorData);
  */
  // read sensor data
  if (checkTemperature)
  {
    readTemperature(false);
    timeoutReadTemperature = now;
  }

  if (wifiConnectionAvailable)
  {
    // Check for Configuration Changes
    if (checkConfiguration)
    {
      tm.checkUpdate(false, &config);
      timeoutCheckConfiguration = now;
      // force chack temperature
      checkReleTemperature = true;
    }
  }

#ifdef FLAGRELETEMP
  if (checkReleTemperature)
  {
    int currentStatus = config.clientStatus;
    bool on = false;

    if (configurationAvailable)
      on = checkThermostatStatus(sensorData.currentTemperature, &config, wifiConnectionAvailable);
    config.clientStatus = on ? STATUS_ON : STATUS_OFF;
    if (on)
      digitalWrite(relayPin, LOW);
    else
      digitalWrite(relayPin, HIGH);
    if (config.clientStatus != currentStatus)
    {
      // force call monitor to update server status
      sendMonitorData = true;
    }
    timeoutSetTemperatureRele = now;
  }
#endif

  if (wifiConnectionAvailable)
  {
    // send Data to monitor
    if (sendMonitorData)
    {
      tm.sendMonitorData(&config, &sensorData);
      // reset temp
      readTemperature(true);
      timeoutCallMonitor = now;
    }
  }
  logger.printlnLog("Check FreeMemory %d", freeMemory());
  delay(WAIT_MAIN_LOOP);
}

/**
  Recupera record di programmazione corrente in base a giorno / ora
*/
void getCurrentProgrammingRecord(PROG_TIME *progRecord, CONFIG *conf)
{
  // get current day and time
  time_t t = tm.getWiFiTime(); //hc.getTime() + ONE_HOUR;
  struct tm *timeinfo;
  timeinfo = localtime(&t);
  logger.printlnLog("GET Programming NOW : day %d, hour %d , minute %d, dayl %d",
                    timeinfo->tm_wday, timeinfo->tm_hour, timeinfo->tm_min,
                    timeinfo->tm_isdst);
  int tnow = timeinfo->tm_hour * 60 + timeinfo->tm_min;
  int day = timeinfo->tm_wday - 1;
  if (day < 0)
    day = 6;
  progRecord->minTemp = conf->minTemp;
  progRecord->priorityDisp = 0;
  for (int i = 0; i < conf->day[day].numProg; i++)
  {
    int ts = conf->day[day].prog[i].timeStart;
    int te = conf->day[day].prog[i].timeEnd;
    if (ts <= tnow && tnow <= te)
    {
      progRecord->minTemp = conf->day[day].prog[i].minTemp;
      progRecord->priorityDisp = conf->day[day].prog[i].priorityDisp;
      break;
    }
  }
  logger.printlnLog("Day %d - Time %d -> Programming Temp %f - IdDispPry %d", day,
                    tnow, progRecord->minTemp, progRecord->priorityDisp);
}

/**
   Verifica temperatura ed in funzione di programmazione determina se accendere o meno
*/
bool checkThermostatStatus(float cT, CONFIG *conf, boolean connectionAvailable)
{
  bool on = false;
  float tempToCheck = 0;
  float managedTemp = 0;
  switch (conf->serverStatus)
  {
  case STATUS_ON:
    on = true;
    break;
  case STATUS_OFF:
    on = false;
    break;
  case STATUS_MANUAL:
    tempToCheck = conf->minTempManual;
    break;
  case STATUS_AUTO:
    PROG_TIME progRecord;
    // recupera record di programmazione temperatura
    getCurrentProgrammingRecord(&progRecord, conf);
    tempToCheck = progRecord.minTemp;
    // recupera temperatura
    if (connectionAvailable)
    {
      managedTemp = getManagedTemperature(progRecord.priorityDisp, conf);
      if (managedTemp > 0)
        cT = managedTemp;
    }
    break;

  default:
    on = false;
    break;
  }
  if (tempToCheck > 0)
    on = cT < tempToCheck;
  logger.printlnLog("Current Temp %f - Temp to Check %f - Status %d : ON = %d",
                    cT, tempToCheck, conf->serverStatus, on);
  return on;
}
/**
   Retrive managed temperature according curent programming and status
*/
float getManagedTemperature(int pryDisp, CONFIG *conf)
{
  // se priorità dispositivo = Default uso dispositivo locale
  if (pryDisp == 0)
    pryDisp = conf->key;
  float tempToCheck = 0.0;
  TEMPDATA tdata;
  tm.getCurrentData(&tdata);
  if (tdata.num > 0)
  {
    switch (tdata.tempMeasure)
    {
    case TEMP_AVARAGE:
      for (int i = 0; i < tdata.num; i++)
        tempToCheck += tdata.data[i].temperature;
      tempToCheck = tempToCheck / (float)tdata.num;
      break;
    case TEMP_PRIORITY:
      for (int i = 0; i < tdata.num; i++)
        if (tdata.data[i].idDisp == pryDisp)
        {
          tempToCheck = tdata.data[i].temperature;
          break;
        }
      break;
    case TEMP_LOCAL:
      for (int i = 0; i < tdata.num; i++)
        if (tdata.data[i].idDisp == conf->key)
        {
          tempToCheck = tdata.data[i].temperature;
          break;
        }
    default:
      break;
    }
  }
  else
    tempToCheck = conf->minTemp;
  return tempToCheck;
}

/**
   Read current sensor data
*/
void readTemperature(boolean init)
{
  if (init)
  {
    sensorData.currentTemperature = 99.0;
    sensorData.totalTemperature = 0.0;
    sensorData.totalPressure = 0.0;
    sensorData.totalLight = 0.0;
    sensorData.humidity = 0.0;
    sensorData.numItem = 0;
  }
  if (BMP280)
  {
    sensorData.numItem++;
    float p = bme.readPressure();
    float t = bme.readTemperature();
    float l = (float)analogRead(sensorPin) / 10.24;
    sensorData.totalTemperature += t;
    sensorData.totalPressure += p;
    sensorData.totalLight += l;
#ifdef BME
    float u = bme.readHumidity();
    sensorData.humidity += u;
#endif
    sensorData.currentTemperature = sensorData.totalTemperature / sensorData.numItem;
    if (false)
      logger.printlnLog(
          "Read Temperature %f - Pressure %f - Light %f - Humidity %f - Medium Temperature %f(%d)",
          t, p, l, u, sensorData.currentTemperature, sensorData.numItem);
  }
}
/*
   Check if I2C address is available
*/
bool checkI2CAddress(int address)
{
  Wire.beginTransmission(address);
  int error = Wire.endTransmission();
  return error == 0;
}

/**
   Check if a configuration is available
*/
bool checkConfiguration(CONFIG *conf, bool connectionAvailable)
{
  if (conf->key == 0)
  {
    if (connectionAvailable)
    {
      logger.printlnLog("No configuration available.. call WiFiRegister");
      tm.wiFiRegister(conf);
    }
    else
    {
      logger.printlnLog("No configuration available and No Connection available !!");
    }
  }
  return conf->key != 0;
}

/**
   Manage LCD output
*/
void displayStatus()
{
  // Get Time
  time_t t = tm.getWiFiTime(); //hc.getTime() + ONE_HOUR;
  struct tm *timeinfo;
  timeinfo = localtime(&t);
  char buffer[80];
  strftime(buffer, 80, "%d-%m-%Y %H:%M:%S", timeinfo);
  // get ip adress
  char lcdBuffer[3 * 4 + 2];
  wifi.getLocalIp(lcdBuffer);
  logger.printlnLog("Check at %s - IP : %s - FreeMemory %d", buffer, lcdBuffer, freeMemory());
  if (LCD)
  {
    char line[20];

    lcd.setCursor(0, 0);
    lcd.print("TermClient");
    // display ip adress
    lcd.setCursor(0, 1);
    sprintf(line, "IP %s", lcdBuffer);
    lcd.print(line);
    // display time from WIFI
    lcd.setCursor(0, 2);
    sprintf(line, "%s", buffer);
    lcd.print(line);
    if (BMP280)
    {
      lcd.createChar(0, bell);
      char c = (char)223;
      char c1 = (char)0;
      float t;
      if (sensorData.numItem > 0)
        t = sensorData.totalTemperature / sensorData.numItem;
      else
        t = bme.readTemperature();
      sprintf(line, "Temp %3.3f %cC  %c", t, c, c1);
      lcd.setCursor(0, 3);
      lcd.print(line);
      lcd.write(0);
    }
  }
  else
  {
  }
}

void onMqttMessage(int messageSize)
{
  // we received a message, print out the topic and contents
  Serial.print("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', duplicate = ");
  Serial.print(mqttClient.messageDup() ? "true" : "false");
  Serial.print(", QoS = ");
  Serial.print(mqttClient.messageQoS());
  Serial.print(", retained = ");
  Serial.print(mqttClient.messageRetain() ? "true" : "false");
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  char messageTopic[100];
  strcpy(messageTopic, mqttClient.messageTopic().c_str());
  if (strcmp(messageTopic, TOPIC_UPDATEPROG) == 0)
  {
    // process update configuration
    char message[messageSize];
    mqttClient.read((uint8_t *)message, messageSize);
    logger.printlnLog("Letto da Topic %s messaggio %s", messageTopic, message);
    messageParser.updateConfigurationResponse(config,message);
  } else if (strcmp(messageTopic, TOPIC_UPDATETEMP) == 0)
  {
    char message[messageSize];    
    mqttClient.read((uint8_t *)message, messageSize);
    logger.printlnLog("Letto da Topic %s messaggio %s", messageTopic, message);
    
  } else
  {
    logger.printlnLog("Letto da Topic %s messaggio non implementato", messageTopic);
  }
}
