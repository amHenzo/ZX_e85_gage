#include "app_state.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
SemaphoreHandle_t stateMutex = nullptr;
ZxGaugeState state;
}

void appStateInit()
{
  if (stateMutex == nullptr) {
    stateMutex = xSemaphoreCreateMutex();
  }

  memset(&state, 0, sizeof(state));
  strncpy(state.timeText, "--:--:--", sizeof(state.timeText));
  strncpy(state.dateText, "--/--/----", sizeof(state.dateText));
  state.spriteIndex = 2;
}

void appStateGet(ZxGaugeState &out)
{
  if (stateMutex != nullptr) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
  }
  out = state;
  if (stateMutex != nullptr) {
    xSemaphoreGive(stateMutex);
  }
}

void appStateUpdate(const ZxGaugeState &next)
{
  if (stateMutex != nullptr) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
  }
  state = next;
  if (stateMutex != nullptr) {
    xSemaphoreGive(stateMutex);
  }
}

void appStateSetFps(float fps)
{
  if (stateMutex != nullptr) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
  }
  state.fps = fps;
  if (stateMutex != nullptr) {
    xSemaphoreGive(stateMutex);
  }
}

void appStateSetBootComplete(bool complete)
{
  if (stateMutex != nullptr) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
  }
  state.bootComplete = complete;
  if (stateMutex != nullptr) {
    xSemaphoreGive(stateMutex);
  }
}

void appStateSetPortalReady(bool ready)
{
  if (stateMutex != nullptr) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
  }
  state.portalReady = ready;
  if (stateMutex != nullptr) {
    xSemaphoreGive(stateMutex);
  }
}

void appStateSetTime(const char *timeText, const char *dateText, bool configured)
{
  if (stateMutex != nullptr) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
  }
  strncpy(state.timeText, timeText, sizeof(state.timeText) - 1);
  state.timeText[sizeof(state.timeText) - 1] = '\0';
  strncpy(state.dateText, dateText, sizeof(state.dateText) - 1);
  state.dateText[sizeof(state.dateText) - 1] = '\0';
  state.rtcConfigured = configured;
  if (stateMutex != nullptr) {
    xSemaphoreGive(stateMutex);
  }
}

void appStateSetSensorData(float temperatureC, float humidityPct, float pitch, float roll, uint8_t spriteIndex)
{
  if (stateMutex != nullptr) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
  }
  state.temperatureC = temperatureC;
  state.humidityPct = humidityPct;
  state.pitch = pitch;
  state.roll = roll;
  state.spriteIndex = spriteIndex;
  if (stateMutex != nullptr) {
    xSemaphoreGive(stateMutex);
  }
}
