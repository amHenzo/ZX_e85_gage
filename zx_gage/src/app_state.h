#ifndef APP_STATE_H
#define APP_STATE_H

#include <Arduino.h>

struct ZxGaugeState {
  char timeText[9];
  char dateText[11];
  float temperatureC;
  float humidityPct;
  float pitch;
  float roll;
  uint8_t spriteIndex;
  float fps;
  bool bootComplete;
  bool portalReady;
  bool rtcConfigured;
};

void appStateInit();
void appStateGet(ZxGaugeState &out);
void appStateUpdate(const ZxGaugeState &next);
void appStateSetFps(float fps);
void appStateSetBootComplete(bool complete);
void appStateSetPortalReady(bool ready);
void appStateSetTime(const char *timeText, const char *dateText, bool configured);
void appStateSetSensorData(float temperatureC, float humidityPct, float pitch, float roll, uint8_t spriteIndex);

#endif
