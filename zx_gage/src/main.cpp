#include <Arduino.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_state.h"
#include "display.h"
#include "hardware_config.h"
#include "loading.h"
#include "sensors.h"
#include "spi_bus.h"
#include "time_service.h"
#include "web_portal.h"

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_ONBOARD_LED, OUTPUT);
  digitalWrite(PIN_ONBOARD_LED, HIGH);

  appStateInit();
  spiBusInit();

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);

  displayInit();
  loadingShowStage(10, 220);

  sensorsInit();
  loadingShowStage(35, 220);

  timeServiceInit();
  loadingShowStage(60, 220);

  webPortalStart();
  loadingShowStage(80, 220);

  sensorsStartTask();
  timeServiceStartTask();
  appStateSetBootComplete(true);
  loadingShowStage(100, 350);

  displayStartTask();
  digitalWrite(PIN_ONBOARD_LED, LOW);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
