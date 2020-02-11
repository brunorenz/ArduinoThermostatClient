#ifndef Logging_h
#define Logging_h

#include "TermClient.h"
#include <stdarg.h>
#include <stdio.h>
//#include "hardwareSerial.h"

#ifdef ARDUINO_MKR1000
#include <RTCZero.h>
#endif
#include <time.h>

#define LOG_BUFFER_SIZE 500
#define MALLOC_
class Logging
{
public:
  Logging();
  #ifdef ARDUINO_MKR1000
  Logging(RTCZero *rtc);
  void setRTC(RTCZero *rtc);
  #endif
  void printLog(const char *format, ...);
  void printlnLog(const char *format, ...);
  
  bool isLogEnabled();

private:
  bool available = false;
  #ifdef ARDUINO_MKR1000
  RTCZero *__rtc;
  void formatData(char *buffer);  
  #endif
  
  char bufferData[80];

#ifndef MALLOC
  char printBuffer[LOG_BUFFER_SIZE];
#endif
};
#endif
