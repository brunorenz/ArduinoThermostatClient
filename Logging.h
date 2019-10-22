#ifndef Logging_h
#define Logging_h

#include "TermClient.h"
#include <Arduino.h>
#include <stdarg.h>
#include <RTCZero.h>
#include <time.h>

#define LOG_BUFFER_SIZE 300
#define MALLOC
class Logging
{
public:
  Logging();
  Logging(RTCZero *rtc);
  template<class ...Args>
  void printLog(const char *format, Args ... args);
  void printlnLog(const char *format, ...);
  void setRTC(RTCZero *rtc);

private:
  void printLog(const char *format, boolean nl , ...);
  bool available = false;
  RTCZero *__rtc;
  void formatData(char *buffer);
  char bufferData[80];

#ifndef MALLOC
  char printBuffer[LOG_BUFFER_SIZE];
#endif
  //void _printlnLog(bool ln,const char * format, ...);
};
#endif
