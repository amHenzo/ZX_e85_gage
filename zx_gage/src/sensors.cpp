#include "sensors.h"

#include <DHT.h>
#include <SPI.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_state.h"
#include "hardware_config.h"
#include "spi_bus.h"

namespace {
constexpr uint8_t DHT_TYPE = DHT11;
constexpr uint8_t REG_POWER_CTL = 0x2D;
constexpr uint8_t REG_DATA_FORMAT = 0x31;
constexpr uint8_t REG_DATAX0 = 0x32;
constexpr float ADXL_G_PER_LSB = 0.0078f;

DHT dht(PIN_DHT, DHT_TYPE);
SensorSample sample = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2};
float rawRoll = 0.0f;
float rawPitch = 0.0f;
unsigned long lastDhtRead = 0;

void writeRegister(uint8_t registerAddress, uint8_t value)
{
  SpiBusLock lock;
  digitalWrite(PIN_ADXL_CS, LOW);
  SPI.transfer(registerAddress);
  SPI.transfer(value);
  digitalWrite(PIN_ADXL_CS, HIGH);
}

void readRegister(uint8_t registerAddress, uint8_t numBytes, uint8_t *values)
{
  uint8_t address = 0x80 | registerAddress;
  if (numBytes > 1) {
    address |= 0x40;
  }

  SpiBusLock lock;
  digitalWrite(PIN_ADXL_CS, LOW);
  SPI.transfer(address);
  for (uint8_t i = 0; i < numBytes; i++) {
    values[i] = SPI.transfer(0x00);
  }
  digitalWrite(PIN_ADXL_CS, HIGH);
}

void sensorTask(void *)
{
  unsigned long lastRtcPush = 0;
  for (;;) {
    sensorsUpdateFast();
    sensorsUpdateSlow();

    const unsigned long now = millis();
    if (now - lastRtcPush >= 100) {
      appStateSetSensorData(sample.temperatureC, sample.humidityPct, sample.accelXG, sample.accelYG, sample.accelZG, sample.pitch, sample.roll, sample.spriteIndex);
      lastRtcPush = now;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
}

void sensorsInit()
{
  dht.begin();
  pinMode(PIN_ADXL_CS, OUTPUT);
  digitalWrite(PIN_ADXL_CS, HIGH);

  writeRegister(REG_DATA_FORMAT, 0x01);
  writeRegister(REG_POWER_CTL, 0x08);
}

void sensorsUpdateFast()
{
  uint8_t values[6];
  readRegister(REG_DATAX0, 6, values);

  const int16_t x = ((int16_t)values[1] << 8) | values[0];
  const int16_t y = ((int16_t)values[3] << 8) | values[2];
  const int16_t z = ((int16_t)values[5] << 8) | values[4];

  sample.accelXG = x * ADXL_G_PER_LSB;
  sample.accelYG = y * ADXL_G_PER_LSB;
  sample.accelZG = z * ADXL_G_PER_LSB;

  rawRoll = atan2f((float)y, sqrtf(((float)x * x) + ((float)z * z))) * 180.0f / PI;
  rawPitch = atan2f((float)-x, sqrtf(((float)y * y) + ((float)z * z))) * 180.0f / PI;

  sample.roll = (0.85f * sample.roll) + (0.15f * rawRoll);
  sample.pitch = (0.85f * sample.pitch) + (0.15f * rawPitch);
  sample.spriteIndex = sensorsSpriteFor(sample.pitch, sample.roll);
}

void sensorsUpdateSlow()
{
  const unsigned long now = millis();
  if (now - lastDhtRead < 2000) {
    return;
  }

  const float humidity = dht.readHumidity();
  const float temperature = dht.readTemperature();
  if (!isnan(humidity)) {
    sample.humidityPct = humidity;
  }
  if (!isnan(temperature)) {
    sample.temperatureC = temperature;
  }
  lastDhtRead = now;
}

SensorSample sensorsCurrent()
{
  return sample;
}

uint8_t sensorsSpriteFor(float pitch, float roll)
{
  if (fabsf(pitch) < 15.0f && fabsf(roll) < 15.0f) {
    return 2;
  }

  if (fabsf(pitch) > fabsf(roll)) {
    return pitch < 0 ? 1 : 3;
  }

  return roll < 0 ? 0 : 4;
}

void sensorsStartTask()
{
  xTaskCreatePinnedToCore(sensorTask, "sensors", 4096, nullptr, 2, nullptr, 0);
}
