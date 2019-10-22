#ifndef TermClient_h
#define TermClient_h

/**
  TermClient constant header
*/
#define WEBSOCKET_
#define REST_OFF
#define LOCAL_

//static char TERM_SERVER_URL[] = "192.168.0.106";  // server address
//int TERM_SERVER_PORT = 8100;

#define ONE_HOUR 3600
#define TERM_SERVER_URL_REMOTE "192.168.0.119"
#define TERM_SERVER_URL_LOCAL "192.168.0.119"
#ifdef LOCAL
#define TERM_SERVER_URL TERM_SERVER_URL_LOCAL
#else
#define TERM_SERVER_URL TERM_SERVER_URL_REMOTE
#endif
//#define TERM_SERVER_URL_REMOTE       "51.68.197.97"
#define TERM_SERVER_PORT 8100

#define SECRET_SSID "BRHome"
#define SECRET_PASS "ba192hbA:ah060vyA"
//
#define TERM_ID_CUST 1
#define TERM_ID_DISP 1

#define CONNECT_WAIT_TIME 5000
// LCD
#define ADDRESS_LCD 0x3f
#define ADDRESS_BMP280 0x76
#define ADDRESS_BME280 0x76

#define MYDEBUG
#define BME
#define NO_BMP

#define GET_TIMEOUT 6000
#define GET_JSON_BUFFER 3000

#define WAIT_CALL_CHECK 5000L
//#define WAIT_CALL_CHECK       10000L
#define WAIT_CALL_MONITOR 30000L
//#define WAIT_CALL_MONITOR     6000L
#define WAIT_READ_TEMPERATURE 5000L
#define WAIT_SETRELE_TEMPERATURE 20000L
#define WAIT_MAIN_LOOP 1000L

#define REST_RET_OK 0
#define REST_RET_WARN 4
#define REST_RET_ERROR 8

#define STATUS_OFF 0
#define STATUS_ON 1
#define STATUS_AUTO 2
#define STATUS_MANUAL 3

#define HTTP_HEADPOST "User-Agent: ArduinoMKR1000\r\nContent-Type: application/json\r\nConnection: keep-alive"
#define HTTP_HEADGET "User-Agent: ArduinoMKR1000\r\nConnection: keep-alive"
//#define HTTP_HEADGET "User-Agent: ArduinoMKR1000\r\nContent-Type: application/x-www-form-urlencoded\r\nConnection: Close\r\n"
#define HTTP_UA "User-Agent: ArduinoMKR1000"
#define HTTP_CT "Content-Type: application/json"
#define HTTP_ENC "Accept: */*"
#define GET_REGISTER "/therm/rest/wifiRegisterG"
#define GET_CHECK "/therm/rest/checkConfigurationChange"
#define GET_CURRENTDATA "/therm/rest/getCurrentData"
#define POST_MONITOR "/therm/rest/monitor"
#define POST_REGISTER "/therm/rest/wifiRegister"

#define MAINCONTROLLER
#define LIGTH

#ifdef LIGTH
#define FLAGLIGHT
#ifdef MOTION
#define FLAGMOTION
#define FLAGRELEMOTION
#endif
#else
#endif

#ifdef MAINCONTROLLER
#define FLAGRELETEMP
#else
#endif

#define MAX_DEVICE 4
#define MAX_PROGDAY 4
#define MAX_DAY 7

#define TEMP_LOCAL 1
#define TEMP_AVARAGE 2
#define TEMP_PRIORITY 3

struct HTTPREQ
{
  char *host;
  int port;
  char *url;
  char *postData;
};

struct PROG_TIME
{
  float minTemp;
  int timeStart;
  int timeEnd;
  int priorityDisp;
};

struct SENSORDATA
{
  float currentTemperature;
  float totalTemperature;
  float totalPressure;
  float totalLight;
  float humidity;
  int numItem;
};

// DEBUG
struct PROGRAM_DAY
{
  int day;
  int numProg;
  struct PROG_TIME
  {
    float minTemp;
    int timeStart;
    int timeEnd;
    int priorityDisp;
  } prog[MAX_PROGDAY];
};

struct CONFIG
{
  int key;
  float minTemp;
  float minTempManual;
  int serverStatus;
  int clientStatus;
  int tempMeasure;
  int flagLcd;
  int flagLightSensor;
  int flagMotionSensor;
  int flagReleTemp;
  int flagReleLight;
  bool progLoaded;
  long lastUpdate;
  unsigned long lastAccess;
  PROGRAM_DAY day[MAX_DAY];
};

struct TEMPDATA
{
  int tempMeasure;
  int num;
  long now;
  struct TEMPD
  {
    int idDisp;
    float temperature;
    long tmsUpd;
  } data[MAX_DEVICE];
};

#endif
