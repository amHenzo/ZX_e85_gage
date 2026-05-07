#include "time_service.h"

#include <DS3231.h>
#include <Wire.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_state.h"
#include "hardware_config.h"

namespace {
RTClib rtc;
DS3231 clockChip;

bool isLeapYear(uint16_t year)
{
  return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

uint8_t daysInMonth(uint16_t year, uint8_t month)
{
  static const uint8_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (month == 2 && isLeapYear(year)) {
    return 29;
  }
  return days[month - 1];
}

uint8_t dayOfWeek(const ManualTime &time)
{
  uint16_t y = time.year;
  uint8_t m = time.month;
  if (m < 3) {
    m += 12;
    y--;
  }

  const uint16_t k = y % 100;
  const uint16_t j = y / 100;
  const uint8_t h = (time.day + ((13 * (m + 1)) / 5) + k + (k / 4) + (j / 4) + (5 * j)) % 7;
  return ((h + 5) % 7) + 1;
}

void timeTask(void *)
{
  for (;;) {
    timeServiceUpdateState();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
}

void timeServiceInit()
{
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  clockChip.setClockMode(false);
  timeServiceUpdateState();
}

void timeServiceUpdateState()
{
  DateTime date = rtc.now();
  char timeText[9];
  char dateText[11];
  snprintf(timeText, sizeof(timeText), "%02d:%02d:%02d", date.hour(), date.minute(), date.second());
  snprintf(dateText, sizeof(dateText), "%02d/%02d/%04d", date.day(), date.month(), date.year());
  appStateSetTime(timeText, dateText, true);
}

bool timeServiceSetManual(const ManualTime &time)
{
  if (!timeServiceValidate(time)) {
    return false;
  }

  clockChip.setClockMode(false);
  clockChip.setSecond(time.second);
  clockChip.setMinute(time.minute);
  clockChip.setHour(time.hour);
  clockChip.setDoW(dayOfWeek(time));
  clockChip.setDate(time.day);
  clockChip.setMonth(time.month);
  clockChip.setYear((uint8_t)(time.year >= 2000 ? time.year - 2000 : time.year));
  timeServiceUpdateState();
  return true;
}

bool timeServiceValidate(const ManualTime &time)
{
  if (time.year < 2020 || time.year > 2099) {
    return false;
  }
  if (time.month < 1 || time.month > 12) {
    return false;
  }
  if (time.day < 1 || time.day > daysInMonth(time.year, time.month)) {
    return false;
  }
  if (time.hour > 23 || time.minute > 59 || time.second > 59) {
    return false;
  }
  return true;
}

void timeServiceStartTask()
{
  xTaskCreatePinnedToCore(timeTask, "time", 3072, nullptr, 1, nullptr, 0);
}
