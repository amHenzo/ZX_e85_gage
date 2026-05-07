#include "display.h"

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_state.h"
#include "Car_texture.h"
#include "hardware_config.h"
#include "spi_bus.h"

namespace {
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, PIN_LCD_CS, PIN_LCD_RESET);
unsigned long fpsWindowStart = 0;
uint16_t frames = 0;
constexpr int16_t CACHE_EMPTY = INT16_MIN;
int16_t cachedTempDeci = CACHE_EMPTY;
int16_t cachedHumidityDeci = CACHE_EMPTY;
int16_t cachedFpsDeci = CACHE_EMPTY;
int16_t cachedFrontCenti = CACHE_EMPTY;
int16_t cachedRearCenti = CACHE_EMPTY;
int16_t cachedLeftCenti = CACHE_EMPTY;
int16_t cachedRightCenti = CACHE_EMPTY;
char cachedTempText[10] = "--.-C";
char cachedHumidityText[10] = "--.-%H";
char cachedFpsText[10] = "--.-FPS";
char cachedFrontText[6] = "0.00";
char cachedRearText[6] = "0.00";
char cachedLeftText[6] = "0.00";
char cachedRightText[6] = "0.00";
constexpr uint32_t LCD_TEST_CLOCKS_HZ[] = {
  800000,
  850000,
  900000,
  950000,
  1000000
};
constexpr unsigned long LCD_TEST_STEP_MS = 4000;

float clampG(float value)
{
  if (value < -1.0f) {
    return -1.0f;
  }
  if (value > 1.0f) {
    return 1.0f;
  }
  return value;
}

void updateDeciText(float value, int16_t &cachedValue, char *text, size_t textSize, const char *suffix)
{
  const int16_t rounded = (int16_t)lroundf(value * 10.0f);
  if (rounded == cachedValue) {
    return;
  }

  cachedValue = rounded;
  snprintf(text, textSize, "%d.%d%s", rounded / 10, abs(rounded % 10), suffix);
}

void updateCentiText(float value, int16_t &cachedValue, char *text, size_t textSize)
{
  const int16_t rounded = (int16_t)lroundf(value * 100.0f);
  if (rounded == cachedValue) {
    return;
  }

  cachedValue = rounded;
  snprintf(text, textSize, "%d.%02d", rounded / 100, abs(rounded % 100));
}

uint32_t currentLcdClockHz()
{
  if (!LCD_SPI_CLOCK_TEST) {
    return LCD_SPI_CLOCK_HZ;
  }

  const size_t count = sizeof(LCD_TEST_CLOCKS_HZ) / sizeof(LCD_TEST_CLOCKS_HZ[0]);
  const size_t index = (millis() / LCD_TEST_STEP_MS) % count;
  return LCD_TEST_CLOCKS_HZ[index];
}

void displayTask(void *)
{
  for (;;) {
    displayRenderDashboard();
    taskYIELD();
  }
}
}

void displayInit()
{
  pinMode(PIN_LCD_CS, OUTPUT);
  u8g2.setBusClock(LCD_SPI_CLOCK_HZ);
  u8g2.begin();
  u8g2.setFont(u8g2_font_profont10_mf);
  fpsWindowStart = millis();
}

U8G2 &displayDevice()
{
  return u8g2;
}

void displayRenderDashboard()
{
  ZxGaugeState snapshot;
  appStateGet(snapshot);
  const float lateralG = -snapshot.accelXG;
  const float frontRearG = -snapshot.accelYG;
  const float leftG = lateralG < 0.0f ? -lateralG : 0.0f;
  const float rightG = lateralG > 0.0f ? lateralG : 0.0f;
  const float frontG = frontRearG > 0.0f ? frontRearG : 0.0f;
  const float rearG = frontRearG < 0.0f ? -frontRearG : 0.0f;

  updateDeciText(snapshot.temperatureC, cachedTempDeci, cachedTempText, sizeof(cachedTempText), "C");
  updateDeciText(snapshot.humidityPct, cachedHumidityDeci, cachedHumidityText, sizeof(cachedHumidityText), "%H");
  updateDeciText(snapshot.fps, cachedFpsDeci, cachedFpsText, sizeof(cachedFpsText), "FPS");
  updateCentiText(frontG, cachedFrontCenti, cachedFrontText, sizeof(cachedFrontText));
  updateCentiText(leftG, cachedLeftCenti, cachedLeftText, sizeof(cachedLeftText));
  updateCentiText(rightG, cachedRightCenti, cachedRightText, sizeof(cachedRightText));
  updateCentiText(rearG, cachedRearCenti, cachedRearText, sizeof(cachedRearText));
  const uint32_t lcdClockHz = currentLcdClockHz();

  {
    SpiBusLock lock;
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_profont10_mf);
    u8g2.setFont(u8g2_font_profont10_mf);
    u8g2.setCursor(0, 8);
    u8g2.print(cachedTempText);
    u8g2.setCursor(0, 18);
    u8g2.print(cachedHumidityText);
    u8g2.setCursor(0, 28);
    u8g2.print(cachedFpsText);

    u8g2.setFont(u8g2_font_profont10_mf);
    u8g2.setCursor(74, 8);
    u8g2.print(snapshot.dateText);

    const uint8_t sprite = snapshot.spriteIndex < bitmap_allArray_LEN ? snapshot.spriteIndex : 2;
    u8g2.drawXBM(62, 8, 20, 64, bitmap_allArray[sprite]);

    const int gaugeCx = 72;
    const int gaugeCy = 38;
    const int dotX = gaugeCx + lroundf(clampG(lateralG) * 8.0f);
    const int dotY = gaugeCy - lroundf(clampG(frontRearG) * 18.0f);
    u8g2.drawDisc(dotX, dotY, 2);

    u8g2.setFont(u8g2_font_profont10_mf);
    u8g2.setCursor(64, 20);
    u8g2.print(cachedFrontText);
    u8g2.setCursor(38, 42);
    u8g2.print(cachedLeftText);
    u8g2.setCursor(88, 42);
    u8g2.print(cachedRightText);
    u8g2.setCursor(64, 62);
    u8g2.print(cachedRearText);
    if (LCD_SPI_CLOCK_TEST) {
      u8g2.setCursor(96, 52);
      u8g2.print(lcdClockHz / 1000);
      u8g2.print("k");
    }

    u8g2.setFont(u8g2_font_profont15_mf);
    u8g2.setCursor(0, 54);
    u8g2.print(snapshot.timeText);
    u8g2.setBusClock(lcdClockHz);
    u8g2.sendBuffer();
  }

  frames++;
  const unsigned long now = millis();
  if (now - fpsWindowStart >= 1000) {
    const float newFps = frames * 1000.0f / (now - fpsWindowStart);
    appStateSetFps(newFps);
    frames = 0;
    fpsWindowStart = now;
  }
}

void displayStartTask()
{
  xTaskCreatePinnedToCore(displayTask, "display", 4096, nullptr, 3, nullptr, 1);
}
