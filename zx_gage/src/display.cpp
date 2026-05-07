#include "display.h"

#include <Arduino.h>
#include <SPI.h>
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
    u8g2.setCursor(0, 7);
    u8g2.print("Frame Test");
    u8g2.setCursor(55, 7);
    u8g2.print(frames);

    u8g2.setCursor(0, 15);
    u8g2.print(snapshot.fps, 1);
    u8g2.setCursor(19, 15);
    u8g2.print(" FPS");

    u8g2.setCursor(0, 23);
    u8g2.print(snapshot.humidityPct, 1);
    u8g2.setCursor(25, 23);
    u8g2.print("\xF7""H");

    u8g2.setCursor(0, 31);
    u8g2.print(snapshot.temperatureC, 1);
    u8g2.setCursor(25, 31);
    u8g2.print("\xB0""C");

    u8g2.setCursor(75, 7);
    u8g2.print(snapshot.dateText);

    u8g2.setFont(u8g2_font_profont15_mf);
    u8g2.setCursor(2, 50);
    u8g2.print(snapshot.timeText);

    const uint8_t sprite = snapshot.spriteIndex < bitmap_allArray_LEN ? snapshot.spriteIndex : 2;
    u8g2.drawXBM(70, 10, 20, 64, bitmap_allArray[sprite]);
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
