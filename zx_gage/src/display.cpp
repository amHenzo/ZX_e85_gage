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

void printGValue(const char *label, float value)
{
  u8g2.print(label);
  u8g2.print(value, 2);
  u8g2.print("G");
}

void printCompactGValue(const char *label, float value)
{
  u8g2.print(label);
  u8g2.print(value, 2);
}

void printCompactGNumber(float value)
{
  u8g2.print(value, 2);
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

  {
    SpiBusLock lock;
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_profont10_mf);
    const float lateralG = -snapshot.accelXG;
    const float frontRearG = -snapshot.accelYG;
    const float leftG = lateralG < 0.0f ? -lateralG : 0.0f;
    const float rightG = lateralG > 0.0f ? lateralG : 0.0f;
    const float frontG = frontRearG > 0.0f ? frontRearG : 0.0f;
    const float rearG = frontRearG < 0.0f ? -frontRearG : 0.0f;

    u8g2.setFont(u8g2_font_profont10_mf);
    u8g2.setCursor(0, 8);
    u8g2.print(snapshot.temperatureC, 1);
    u8g2.print("C");
    u8g2.setCursor(0, 18);
    u8g2.print(snapshot.humidityPct, 1);
    u8g2.print("%H");
    u8g2.setCursor(0, 28);
    u8g2.print(snapshot.fps, 1);
    u8g2.print("FPS");

    u8g2.setFont(u8g2_font_profont10_mf);
    u8g2.setCursor(74, 8);
    u8g2.print(snapshot.dateText);

    const uint8_t sprite = snapshot.spriteIndex < bitmap_allArray_LEN ? snapshot.spriteIndex : 2;
    u8g2.drawXBM(62, 6, 20, 64, bitmap_allArray[sprite]);

    const int gaugeCx = 72;
    const int gaugeCy = 36;
    const int dotX = gaugeCx + lroundf(clampG(lateralG) * 8.0f);
    const int dotY = gaugeCy - lroundf(clampG(frontRearG) * 18.0f);
    u8g2.drawDisc(dotX, dotY, 2);

    u8g2.setFont(u8g2_font_4x6_mf);
    u8g2.setCursor(64, 18);
    printCompactGNumber(frontG);
    u8g2.setCursor(42, 39);
    printCompactGNumber(leftG);
    u8g2.setCursor(88, 39);
    printCompactGNumber(rightG);
    u8g2.setCursor(78, 62);
    printCompactGNumber(rearG);

    u8g2.setFont(u8g2_font_profont15_mf);
    u8g2.setCursor(0, 62);
    u8g2.print(snapshot.timeText);
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
