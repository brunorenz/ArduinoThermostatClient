#ifndef Logging_h
#define Logging_h

#include "TermClient.h"
//#include <Arduino.h>
#include <stdarg.h>
#include <RTCZero.h>
#include <time.h>

#define LOG_BUFFER_SIZE 300
#define MALLOC_
class Logging
{
public:
  Logging();
  Logging(RTCZero *rtc);
  void printLog(const char *format, ...);
  void printlnLog(const char *format, ...);
  void setRTC(RTCZero *rtc);
  bool isLogEnabled();

private:
  //void printLog(const char *format, boolean nl , ...);
  //void printLog(bool nl, const char *format, va_list &args);
  bool available = false;
  RTCZero *__rtc;
  void formatData(char *buffer);
  char bufferData[80];

#ifndef MALLOC
  char printBuffer[LOG_BUFFER_SIZE];
#endif
};
#endif
