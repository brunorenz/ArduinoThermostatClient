/*
  TermClient main module
*/

#include <Adafruit_SleepyDog.h>
#include "TermClient.h"
#include "HttpConnection.h"
//#include "ThermManager.h"
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
#ifdef MQTT_0
#include <ArduinoMqttClient.h>
#endif
#ifdef MQTT_1
#include <MQTT.h>
#endif

//#include <Adafruit_Sensor.h>

int status = WL_IDLE_STATUS; // the WiFi radio's status
boolean LCD = false;
boolean BMP280 = false;
//char temp[250];

//ThermManager tm;
#ifdef USE_MQTT

#else
HttpConnection hc;
#endif

// configuration data
CONFIG config;
SENSORDATA sensorData;

// LCD
LiquidCrystal_I2C lcd(ADDRESS_LCD, 20, 4);

// Temperature Sensor
#ifdef BMP
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bme;
#endif
#ifdef BME
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
#endif

Logging logger;
RTCZero rtc;

WiFiConnection wifi(&logger);

WiFiClient wifiClient;
#ifdef MQTT_0
MqttClient mqttClient(wifiClient);
#endif
#ifdef MQTT_1
MQTTClient mqttClient(256);
#endif

MessageParser messageParser(&logger);

bool checkI2CAddress(int address);
void readTemperature(boolean init);
float getManagedTemperature(int pryDisp, CONFIG *conf);
bool checkConfiguration(CONFIG *conf, bool connectionAvailable);
bool checkThermostatStatus(float cT, CONFIG &conf, boolean connectionAvailable);
void loopREST();
void initConfiguration(CONFIG &cfg);

long timeoutCallMonitor;
long timeoutReadTemperature;
long timeoutSetTemperatureRele;
long timeoutCheckConfiguration;

const int sensorPin = A1;
const int relayPin = 9;
int count = 0;
bool willSent = 0;

// GEstione LCD
uint8_t bell[8] = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};

/**
   Reset function
*/

//void (*resetFunc)(void) = 0; //declare reset function at address 0
/*
uint8_t checkServerConnection(WiFiClient &client)
{
  uint8_t connected = client.connected();
  return connected;
}
*/
/**
  Setup .. initialization
*/

void setup()
{

  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  // wait serial to start
  delay(2000);

  // initialize configuration
  initConfiguration(config);

  /*
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
  config.flagTemperatureSensor = 0;
  config.flagPressureSensor = 0;
  config.flagHumiditySensor = 0;
  config.timeZoneOffset = -1;
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

#ifdef BMP
  config.flagTemperatureSensor = 1;
  config.flagPressureSensor = 1;
  config.flagHumiditySensor = 0;
#endif

#ifdef BME
  config.flagTemperatureSensor = 1;
  config.flagPressureSensor = 1;
  config.flagHumiditySensor = 1;
#endif
*/
  // initialize I2C comunication
  Wire.begin();
  // Initialize rtc
  rtc.begin();
  logger.setRTC(&rtc);
  // check LCD
  LCD = checkLCD();
  if (LCD)
    config.flagLcd = 1;

  // check LCD and Thermostat
  //LCD = checkI2CAddress(ADDRESS_LCD);
  //BMP280 = checkI2CAddress(ADDRESS_BMP280);
  /*
  if (LCD)
  {
    lcd.init();      //initialize the lcd
    lcd.backlight(); //open the backlight
    config.flagLcd = 1;
  }
  */
  // check Thermostat
  BMP280 = checkBME();
  /*
  if (BMP280)
  {
    int status = bme.begin(ADDRESS_BMP280);
    logger.printlnLog("BME Status %d : ", status);
    Serial.println(bme.sensorID(), 16);
  }
  */
  logger.printlnLog("ThermostatClient start...");
  logger.printlnLog("LCD status    .. %s", LCD ? "OK" : "KO");
  logger.printlnLog("BMP280 status .. %s", BMP280 ? "OK" : "KO");

  pinMode(LED_BUILTIN, OUTPUT);
#ifdef FLAGRELETEMP
  // SET Pin for RELE
  pinMode(relayPin, OUTPUT);
#endif
  // Set resolution for analog read
  analogReadResolution(10);

#ifdef USE_MQTT
  setupMQTT();
#elif defined USE_HTTP
  setupREST();
#endif

  timeoutCallMonitor = millis();
  timeoutReadTemperature = timeoutCallMonitor;
  // call as soon as possible the first time
  timeoutCheckConfiguration = 0;
  timeoutSetTemperatureRele = 0;
  // read initial temperature values
  readTemperature(true);
  // imposto Watchdog Timer a 8 Secondi
  Watchdog.enable(SLEEPYDOG_WAIT_TIME);
}

void setupMQTT()
{
#ifdef MQTT_1
  const char broker[] = TERM_SERVER_MQ;
  int port = 1883;
  mqttClient.begin(broker, port, wifiClient);
  mqttClient.onMessageAdvanced(onMqttMessageAdvanced);
#endif
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
   Check and Manage WiFi connection
*/
bool checkWIFIConnection()
{
  bool rc = wifi.connect();
  if (rc)
  {
    // get IP and MAC address
    if (strlen(config.macAddress) == 0)
      wifi.getMacAddress(config.macAddress, sizeof(config.macAddress));
    if (strlen(config.ipAddress) == 0)
      wifi.getLocalIp(config.ipAddress, sizeof(config.ipAddress));
  }
  return rc;
}
/**
   Check and Manage MQTT Connection
*/
bool checkMQConnection()
{
  bool rc; // = 1;
  if (!mqttClient.connected())
  {
    logger.printlnLog("Connecting to MQTT server ..");
    // set a will message, used by the broker when the connection dies unexpectantly
    // you must know the size of the message before hand, and it must be set before connecting
#ifdef MQTT_1
    sendWillMessage();
    rc = mqttClient.connect(config.macAddress);
    if (!rc)
      logger.printlnLog("MQTT connection failed!");
#endif
#ifdef MQTT_0
    sendWillMessage();
    const char broker[] = TERM_SERVER_MQ;
    int port = 1883;
    rc = mqttClient.connect(broker, port);
    if (rc)
    {
      logger.printlnLog("Register MQTT listener ..");
      mqttClient.onMessage(onMqttMessage);
    }
    else
      logger.printlnLog("MQTT connection failed! Error code = %d", mqttClient.connectError());
#endif

    if (rc)
    {
      // subscribe topic
      int subscribeQos = 1;
#if defined FLAGRELETEMP || defined FLAGRELEMOTION
      const char topic1[] = TOPIC_UPDATEPROG;
      mqttClient.subscribe(topic1, subscribeQos);
#endif
#ifdef FLAGRELETEMP
      const char topic2[] = TOPIC_UPDATETEMP;
      mqttClient.subscribe(topic2, subscribeQos);
#endif
    }
  }
  return rc; // == 1;
}

void loopMQ()
{
  long now = millis();
  //bool checkTemperature = (now - timeoutReadTemperature) > WAIT_READ_TEMPERATURE;

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
      bool checkSendMonitorData = (now - timeoutCallMonitor) > WAIT_CALL_MONITOR;
      if (checkSendMonitorData)
      {
        sendMonitorData(config, sensorData);

        // reset temp
        readTemperature(true);
        timeoutReadTemperature = now;
        timeoutCallMonitor = now;
      }
    }
  }
  if (wifiConnectionAvailable)
  {
    //wifi.updateRTC(rtc, config.timeZoneOffset);
    /*
    unsigned long now = wifi.getTime();
    if (now > 0)
    {
      now -= config.timeZoneOffset * 60;
      rtc.setEpoch(now);
    }
    logger.printlnLog("Time %d - offset %d",now,config.timeZoneOffset);
    */
  }

  // read sensor data
  bool checkTemperature = (now - timeoutReadTemperature) > WAIT_READ_TEMPERATURE;
  if (checkTemperature)
  {
    readTemperature(false);
    timeoutReadTemperature = now;
  }

#ifdef FLAGRELETEMP
  bool checkReleTemperature = (now - timeoutSetTemperatureRele) > WAIT_SETRELE_TEMPERATURE;
  if (checkReleTemperature)
  {
    int currentStatus = config.clientStatus;
    bool on = false;

    on = checkThermostatStatus(sensorData.currentTemperature, config, wifiConnectionAvailable);
    config.clientStatus = on ? STATUS_ON : STATUS_OFF;
    if (on)
      digitalWrite(relayPin, LOW);
    else
      digitalWrite(relayPin, HIGH);
    if (config.clientStatus != currentStatus)
    {
      // force call monitor to update server status
      timeoutCallMonitor = 0;
    }
    timeoutSetTemperatureRele = now;
  }
#endif

  logger.printlnLog("Check FreeMemory %d", freeMemory());

#ifdef MQTT_1
  mqttClient.loop();
#endif
#ifdef MQTT_0
  mqttClient.poll();
#endif

  delay(WAIT_MAIN_LOOP);
  // Watchdog Timer
  Watchdog.reset();

  // check WiFi Connection
  // if true check MQ Connection
  // if true WifI WiFiRegister
  // if (config not found  getConnection)
  // subscribe (updatePrograming and updateThemperature)
}
/**
   Send Will message
 **/
void sendWillMessage()
{
  // set a will message, used by the broker when the connection dies unexpectantly
  // you must know the size of the message before hand, and it must be set before connecting
  char willPayload[100];
  sprintf(willPayload, "{\"macAddress\":\"%s\"}", config.macAddress);
  bool willRetain = true;
  int willQos = 1;
  char willTopic[] = TOPIC_LASTWILL;

#ifdef MQTT_1
  mqttClient.setWill(willTopic, willPayload, willRetain, willQos);
#endif
#ifdef MQTT_0
  mqttClient.beginWill(willTopic, strlen(willPayload), willRetain, willQos);
  mqttClient.print(willPayload);
  mqttClient.endWill();
#endif
  logger.printlnLog("Send Will message %s", willPayload);
  willSent = 1;
}
/*
void sendMonitorDataString(CONFIG &cfg, SENSORDATA &sensor)
{
  DynamicJsonDocument jsonBuffer(GET_JSON_BUFFER);
  bool send = messageParser.preparaMonitorDataRequest(cfg, sensor, jsonBuffer);
  if (send)
  {
    int jsonMessageLen = measureJson(jsonBuffer);
    char jsonMessage[jsonMessageLen + 1];
    serializeJson(jsonBuffer, jsonMessage, sizeof(jsonMessage));
    char outTopic[] = TOPIC_MONITOR;
    publishMessage(jsonMessage, outTopic);
  }
}
*/
void sendMonitorDataMQTT(CONFIG &cfg, SENSORDATA &sensor)
{
  DynamicJsonDocument jsonBuffer(GET_JSON_BUFFER);
  bool send = messageParser.preparaMonitorDataRequest(cfg, sensor, jsonBuffer);
  if (send)
  {
    int jsonMessageLen = measureJson(jsonBuffer);
    char jsonMessage[jsonMessageLen + 1];
    serializeJson(jsonBuffer, jsonMessage, sizeof(jsonMessage));
    char outTopic[] = TOPIC_MONITOR;
    publishMessage(jsonMessage, outTopic);
  }
}

void sendMonitorData(CONFIG &cfg, SENSORDATA &sensor)
{

  //char jsonMessage[] = "{\"macAddress\":\"F8:F0:05:F7:DC:49\",\"temperature\":20.79833,\"pressure\":1013.37,\"light\":53.09245,\"humidity\":65.03451,\"statusThermostat\":0,\"numSurveys\":0}";
  char jsonMessage[200];

  float t = sensor.totalTemperature / sensor.numItem;
  float p = sensor.totalPressure / sensor.numItem / 100.0;
  float l = sensor.totalLight / sensor.numItem;
  float h = sensor.humidity / sensor.numItem;
  sprintf(jsonMessage, "{\"macAddress\":\"%s\",\"temperature\": %f,\"pressure\": %f,\"light\": %f,\"humidity\": %f,\"numSurveys\":%d}",
          cfg.macAddress, t, p, l, h, sensor.numItem);
  char outTopic[] = TOPIC_MONITOR;
  publishMessage(jsonMessage, outTopic);
}

/**
   Send WiFi register message

*/
void sendWiFiRegisterMessage(CONFIG &cfg)
{
  DynamicJsonDocument jsonBuffer(GET_JSON_BUFFER);
  messageParser.preparaWiFiRegisterRequest(cfg, jsonBuffer);
  int jsonMessageLen = measureJson(jsonBuffer);
  char jsonMessage[jsonMessageLen + 1];
  serializeJson(jsonBuffer, jsonMessage, sizeof(jsonMessage));
  char outTopic[] = TOPIC_WIFI;
  publishMessage(jsonMessage, outTopic);
  cfg.registered = 1;
  logger.printlnLog("Send WiFiRegister done!");
}

/**
 * Publish a generic message
 */
void publishMessage(char *message, char *topic)
{
  logger.printlnLog("Send message %s (%d) to Topic %s", message, strlen(message), topic);
  bool retained = false;
  int qos = 1;
  bool dup = false;
#ifdef MQTT_1
  boolean rc = mqttClient.publish(topic, message, retained, qos);
  if (!rc)
    logger.printlnLog("Publush failed !!");
#endif
#ifdef MQTT_0
  mqttClient.beginMessage(topic, strlen(message), retained, qos, dup);
  mqttClient.print(message);
  mqttClient.endMessage();
#endif
}

/**
  Recupera record di programmazione corrente in base a giorno / ora
*/
void getCurrentProgrammingRecord(PROG_TIME &progRecord, CONFIG &conf)
{
  // get current day and time
  //time_t t = tm.getWiFiTime(); //hc.getTime() + ONE_HOUR;
  time_t t = rtc.getEpoch();
  struct tm *timeinfo;
  timeinfo = localtime(&t);
  logger.printlnLog("GET Programming NOW : day %d, hour %d , minute %d, dayl %d",
                    timeinfo->tm_wday, timeinfo->tm_hour, timeinfo->tm_min,
                    timeinfo->tm_isdst);
  int tnow = timeinfo->tm_hour * 60 + timeinfo->tm_min;
  int day = timeinfo->tm_wday - 1;
  if (day < 0)
    day = 6;
  progRecord.minTemp = conf.minTemp;
  progRecord.priorityDisp = 0;
  for (int i = 0; i < conf.day[day].numProg; i++)
  {
    int ts = conf.day[day].prog[i].timeStart;
    int te = conf.day[day].prog[i].timeEnd;
    if (ts <= tnow && tnow <= te)
    {
      progRecord.minTemp = conf.day[day].prog[i].minTemp;
      progRecord.priorityDisp = conf.day[day].prog[i].priorityDisp;
      break;
    }
  }
  logger.printlnLog("Day %d - Time %d -> Programming Temp %f - IdDispPry %d", day,
                    tnow, progRecord.minTemp, progRecord.priorityDisp);
}

/**
   Verifica temperatura ed in funzione di programmazione determina se accendere o meno
*/
bool checkThermostatStatus(float cT, CONFIG &conf, boolean connectionAvailable)
{
  bool on = false;
  float tempToCheck = 0;
  float managedTemp = 0;
  switch (conf.serverStatus)
  {
  case STATUS_ON:
    on = true;
    break;
  case STATUS_OFF:
    on = false;
    break;
  case STATUS_MANUAL:
    tempToCheck = conf.minTempManual;
    break;
  case STATUS_AUTO:
    PROG_TIME progRecord;
    // recupera record di programmazione temperatura
    getCurrentProgrammingRecord(progRecord, conf);
    tempToCheck = progRecord.minTemp;
    // recupera temperatura
    // if misura = media & numero dispositivi > 1
    if (false)
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
                    cT, tempToCheck, conf.serverStatus, on);
  return on;
}
/**
   Retrive managed temperature according curent programming and status
*/
float getManagedTemperature(int pryDisp, CONFIG &conf)
{
  // se priorità dispositivo = Default uso dispositivo locale
  if (pryDisp == 0)
    pryDisp = conf.key;
  float tempToCheck = 0.0;
  TEMPDATA tdata;
  //FIXME tm.getCurrentData(&tdata);
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
        if (tdata.data[i].idDisp == conf.key)
        {
          tempToCheck = tdata.data[i].temperature;
          break;
        }
    default:
      break;
    }
  }
  else
    tempToCheck = conf.minTemp;
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
    if (true)
      logger.printlnLog(
          "Read Temperature %f - Pressure %f - Light %f - Humidity %f - Medium Temperature %f(%d)",
          t, p, l, u, sensorData.currentTemperature, sensorData.numItem);
    displayStatus();          
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
*/
/**
   Manage LCD output
*/
void displayStatus()
{
  // Get Time
  //time_t t = tm.getWiFiTime(); //hc.getTime() + ONE_HOUR;
  time_t t = rtc.getEpoch();
  struct tm *timeinfo;
  timeinfo = localtime(&t);
  char buffer[80];
  strftime(buffer, 80, "%d-%m-%Y %H:%M:%S", timeinfo);
  // get ip adress
  char lcdBuffer[3 * 4 + 2];
  wifi.getLocalIp(lcdBuffer, sizeof(lcdBuffer));
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

void onMqttMessageAdvanced(MQTTClient *client, char topic[], char payload[], int payload_length)
{
#ifdef MQTT_1
  if (strcmp(topic, TOPIC_UPDATEPROG) == 0)
  {
    // process update configuration
    char message[payload_length + 1];
    memcpy(message, payload, payload_length);
    message[payload_length] = '\0';
    logger.printlnLog("Letto da Topic %s messaggio %s", topic, message);
    messageParser.updateConfigurationResponse(config, message);
    wifi.updateRTC(rtc, config.timeZoneOffset);
  }
  else
  {
    logger.printlnLog("Letto da Topic %s messaggio non implementato", topic);
  }
#endif
}

void onMqttMessage(int messageSize)
{
#ifdef MQTT_0
  // we received a message, print out the topic and contents
  /*
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
  */
  char messageTopic[100];
  //memcpy(p, payload, length);
  strcpy(messageTopic, mqttClient.messageTopic().c_str());
  if (strcmp(messageTopic, TOPIC_UPDATEPROG) == 0)
  {
    // process update configuration
    char message[messageSize + 1];
    mqttClient.read((uint8_t *)message, messageSize);
    message[messageSize] = 0;
    logger.printlnLog("Letto da Topic %s messaggio %s", messageTopic, message);
    messageParser.updateConfigurationResponse(config, message);
    wifi.updateRTC(rtc, config.timeZoneOffset);
    /*
    unsigned long now = wifi.getTime();
    if (now > 0)
    {
      now -= config.timeZoneOffset * 60;
      rtc.setEpoch(now);
    }*/
  }
  else if (strcmp(messageTopic, TOPIC_UPDATETEMP) == 0)
  {
    char message[messageSize + 1];
    mqttClient.read((uint8_t *)message, messageSize);
    message[messageSize] = 0;
    logger.printlnLog("Letto da Topic %s messaggio %s", messageTopic, message);
  }
  else
  {
    logger.printlnLog("Letto da Topic %s messaggio non implementato", messageTopic);
  }
#endif
}
/**
 * Initialize configuration data
 */
void initConfiguration(CONFIG &cfg)
{
  // initialize configuration data
  cfg.progLoaded = false;
  cfg.serverStatus = STATUS_OFF;
  cfg.clientStatus = STATUS_OFF;
  cfg.key = 0;
  cfg.flagLcd = 0;
  cfg.flagLightSensor = 0;
  cfg.flagMotionSensor = 0;
  cfg.flagReleTemp = 0;
  cfg.flagReleLight = 0;
  cfg.flagTemperatureSensor = 0;
  cfg.flagPressureSensor = 0;
  cfg.flagHumiditySensor = 0;
  cfg.timeZoneOffset = -1;
  cfg.registered = false;
  for (int i = 0; i < MAX_DAY; i++)
  {
    cfg.day[i].day = -1;
    cfg.day[i].numProg = 0;
  }
  cfg.macAddress[0] = 0;
  cfg.ipAddress[0] = 0;

#ifdef FLAGLIGHT
  cfg.flagLightSensor = 1;
#endif
#ifdef FLAGMOTION
  cfg.flagMotionSensor = 1;
#endif
#ifdef FLAGRELEMOTION
  cfg.flagReleLight = 1;
#endif
#ifdef FLAGRELETEMP
  cfg.flagReleTemp = 1;
#endif

#ifdef BMP
  cfg.flagTemperatureSensor = 1;
  cfg.flagPressureSensor = 1;
  cfg.flagHumiditySensor = 0;
#endif

#ifdef BME
  cfg.flagTemperatureSensor = 1;
  cfg.flagPressureSensor = 1;
  cfg.flagHumiditySensor = 1;
#endif
}
/**
 * initialize LCD
 **/
bool checkLCD()
{

  bool flagLCD = checkI2CAddress(ADDRESS_LCD);
  if (flagLCD)
  {
    lcd.init();      //initialize the lcd
    lcd.backlight(); //open the backlight
    lcd.display();
    //config.flagLcd = 1;
  }
  return flagLCD;
}

/**
 * initialize BME sensor
 **/
bool checkBME()
{
  bool flagBME = checkI2CAddress(ADDRESS_BMP280);
  if (flagBME)
  {
    int status = bme.begin(ADDRESS_BMP280);
    logger.printlnLog("BME Status %d : ", status);
  }
  return flagBME;
}

#ifdef USE_HTTP
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
#endif
