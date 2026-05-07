#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

struct SensorSample {
  float temperatureC;
  float humidityPct;
  float pitch;
  float roll;
  uint8_t spriteIndex;
};

void sensorsInit();
void sensorsUpdateFast();
void sensorsUpdateSlow();
SensorSample sensorsCurrent();
uint8_t sensorsSpriteFor(float pitch, float roll);
void sensorsStartTask();

#endif
