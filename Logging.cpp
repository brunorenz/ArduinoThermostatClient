#include "Logging.h"

Logging::Logging()
{
  available = Serial;
  __rtc = NULL;
}

Logging::Logging(RTCZero *rtc)
{
  available = Serial;
  __rtc = rtc;
}

void Logging::formatData(char *buffer)
{
  time_t t = __rtc->getEpoch();
  struct tm *timeinfo;
  timeinfo = localtime(&t);

  strftime(buffer, 80, "%d-%m-%Y %H:%M:%S ", timeinfo);
}

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
      if (__rtc != NULL)
      {
        formatData(bufferData);
        Serial.print(bufferData);
      }
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

void Logging::setRTC(RTCZero *rtc)
{
  __rtc = rtc;
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

      vsprintf(printBuffer, format, args);
      va_end(args);
      if (__rtc != NULL)
      {
        formatData(bufferData);
        Serial.print(bufferData);
      }
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
