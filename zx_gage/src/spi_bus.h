#ifndef SPI_BUS_H
#define SPI_BUS_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

void spiBusInit();
SemaphoreHandle_t spiBusMutex();

class SpiBusLock {
public:
  SpiBusLock();
  ~SpiBusLock();

private:
  bool locked;
};

#endif
