#include "loading.h"

#include <math.h>
#include <pgmspace.h>
#include "Car_texture.h"
#include "display.h"
#include "spi_bus.h"

namespace {
constexpr uint8_t CAR_W = 20;
constexpr uint8_t CAR_H = 64;
constexpr uint8_t BYTES_PER_ROW = 3;

bool carPixel(uint8_t x, uint8_t y)
{
  const uint16_t offset = (y * BYTES_PER_ROW) + (x / 8);
  const uint8_t rowByte = pgm_read_byte(&bitmap_car_normal[offset]);
  return (rowByte & (1 << (x & 0x07))) != 0;
}

void drawRotatedCar(U8G2 &u8g2, int cx, int cy, uint16_t angleDeg)
{
  // angleDeg is the car rotation in degrees. 0/90/180/270 turn the bitmap
  // around its center; reverse the sign of angleDeg if you want the spin to go
  // the opposite direction.
  const float angle = angleDeg * PI / 180.0f;
  const float s = sinf(angle);
  const float c = cosf(angle);

  // These define the rotation pivot inside the source bitmap. The current
  // values use the exact center of the 20x64 car. Change sourceCx/sourceCy if
  // you want it to rotate around a different point, like the nose or rear axle.
  const float sourceCx = (CAR_W - 1) * 0.9f;
  const float sourceCy = (CAR_H - 1) * 0.01f;

  // Size of the car while it rotates. Increase this to make the car larger;
  // decrease it if the corners clip the 128x64 screen during rotation.
  const float scale = 0.8f;

  for (uint8_t y = 0; y < CAR_H; y++) {
    for (uint8_t x = 0; x < CAR_W; x++) {
      if (!carPixel(x, y)) {
        continue;
      }

      const float px = (x - sourceCx) * scale;
      const float py = (y - sourceCy) * scale;

      // Standard 2D rotation around the bitmap pivot, then translated to the
      // screen center passed in as cx/cy.
      const int rx = cx + lroundf((px * c) - (py * s));
      const int ry = cy + lroundf((px * s) + (py * c));

      if (rx >= 0 && rx < 128 && ry >= 0 && ry < 64) {
        u8g2.drawPixel(rx, ry);
      }
    }
  }
}
}

void loadingDraw(uint8_t progress, uint16_t angleDeg)
{
  if (progress > 100) {
    progress = 100;
  }

  U8G2 &u8g2 = displayDevice();
  SpiBusLock lock;
  u8g2.clearBuffer();

  // cx/cy place the rotating car on the display. Move these numbers if the
  // rotated car should orbit around a different screen position.
  drawRotatedCar(u8g2, 64, 27, angleDeg);

  u8g2.drawFrame(18, 55, 92, 7);
  const uint8_t fill = map(progress, 0, 100, 0, 88);
  u8g2.drawBox(20, 57, fill, 3);
  u8g2.sendBuffer();
}

void loadingShowStage(uint8_t progress, unsigned long minStageMs)
{
  const unsigned long start = millis();
  do {
    // This controls animation speed and direction. Smaller divisor = faster
    // spin. Use (360 - ((millis() / 4) % 360)) for the opposite direction.
    loadingDraw(progress, (millis() / 7) % 360);
    delay(25);
  } while (millis() - start < minStageMs);
}
