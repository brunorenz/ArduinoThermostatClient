#include "Logging.h"

Logging::Logging()
{
  available = Serial;
  #ifdef ARDUINO_MKR1000
  __rtc = NULL;
  #endif
}


#ifdef ARDUINO_MKR1000
Logging::Logging(RTCZero *rtc)
{
  available = Serial;
  __rtc = rtc;
}

void Logging::setRTC(RTCZero *rtc)
{
  __rtc = rtc;
}
#endif

#ifdef ARDUINO_MKR1000
void Logging::formatData(char *buffer)
{
  if (__rtc != NULL)
  {
    time_t t = __rtc->getEpoch();
    struct tm *timeinfo;
    timeinfo = localtime(&t);

    strftime(buffer, strlen(buffer), "%d-%m-%Y %H:%M:%S ", timeinfo);
  }
  else
    sprintf(buffer, "NOTIME");
}
#endif

bool Logging::isLogEnabled()
{
#ifdef MYDEBUG
  if (!available)
    available = Serial;
#else
  available = false;
#endif
  return available;
}

void Logging::printLog(const char *format, ...)
{
#ifdef MYDEBUG
  if (isLogEnabled())
  {
#ifdef MALLOC
    char *printBuffer = (char *)malloc(LOG_BUFFER_SIZE);
    if (printBuffer != NULL)
    {
#endif
      va_list args;
      va_start(args, format);
      vsnprintf(printBuffer, LOG_BUFFER_SIZE, format, args);
      va_end(args);
      #ifdef ARDUINO_MKR1000
      if (__rtc != NULL)
      {
        formatData(bufferData);
        Serial.print(bufferData);
      }
      #endif
      //Serial.print(sizeof(printBuffer));
      //Serial.print(" ");
      Serial.print(printBuffer);
#ifdef MALLOC
      free(printBuffer);
    }
#endif
  }
#endif
}


void Logging::printlnLog(const char *format, ...)
{
#ifdef MYDEBUG
  if (isLogEnabled())
  {
#ifdef MALLOC
    char *printBuffer = (char *)malloc(LOG_BUFFER_SIZE);
    if (printBuffer != NULL)
    {
#endif
      va_list args;
      va_start(args, format);

      vsnprintf(printBuffer, LOG_BUFFER_SIZE, format, args);
      va_end(args);
      #ifdef ARDUINO_MKR1000
      if (__rtc != NULL)
      {
        formatData(bufferData);
        Serial.print(bufferData);
      }
      #endif
      //Serial.print(strlen(printBuffer));
      //Serial.print(" ");
      Serial.println(printBuffer);
#ifdef MALLOC
      free(printBuffer);
    }
#endif
  }
#endif
}
