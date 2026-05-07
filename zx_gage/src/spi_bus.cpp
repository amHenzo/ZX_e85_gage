#include "spi_bus.h"

namespace {
SemaphoreHandle_t busMutex = nullptr;
}

void spiBusInit()
{
  if (busMutex == nullptr) {
    busMutex = xSemaphoreCreateMutex();
  }
}

SemaphoreHandle_t spiBusMutex()
{
  return busMutex;
}

SpiBusLock::SpiBusLock() : locked(false)
{
  if (busMutex != nullptr) {
    locked = xSemaphoreTake(busMutex, portMAX_DELAY) == pdTRUE;
  }
}

SpiBusLock::~SpiBusLock()
{
  if (locked && busMutex != nullptr) {
    xSemaphoreGive(busMutex);
  }
}
