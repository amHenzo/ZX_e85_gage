#ifndef DISPLAY_CONFIG_H
#define DISPLAY_CONFIG_H

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// Display pins and configuration
#define DISPLAY_CS 5

// U8g2 instance
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, /* CS=*/ DISPLAY_CS, /* reset=*/ 22);

// Initialize display
void initDisplay() {
    // lcd 800000
    u8g2.setBusClock(800000);
    u8g2.begin();
    u8g2.setFont(u8g2_font_profont10_mf);
    Serial.println("[Display] U8g2 ST7920 128x64 initialized");
}

// Clear display
void clearDisplay() {
    u8g2.clearBuffer();
}

// Send buffer to display
void updateDisplay() {
    u8g2.sendBuffer();
}

// Get display width
uint8_t getDisplayWidth() {
    return u8g2.getDisplayWidth();
}

// Get display height
uint8_t getDisplayHeight() {
    return u8g2.getDisplayHeight();
}

#endif
