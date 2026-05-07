#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <Arduino.h>

constexpr uint8_t PIN_I2C_SDA = 4;
constexpr uint8_t PIN_I2C_SCL = 21;

constexpr uint8_t PIN_LCD_CS = 5;
constexpr uint8_t PIN_LCD_RESET = 22;
constexpr uint8_t PIN_ADXL_CS = 16;
constexpr uint8_t PIN_DHT = 17;
constexpr uint8_t PIN_ONBOARD_LED = 2;

constexpr uint32_t LCD_SPI_CLOCK_HZ = 900000;
constexpr bool LCD_SPI_CLOCK_TEST = false;

#endif
