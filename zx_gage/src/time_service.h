#ifndef TIME_SERVICE_H
#define TIME_SERVICE_H

#include <Arduino.h>

struct ManualTime {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

void timeServiceInit();
void timeServiceUpdateState();
bool timeServiceSetManual(const ManualTime &time);
bool timeServiceValidate(const ManualTime &time);
void timeServiceStartTask();

#endif
