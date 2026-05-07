#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <Arduino.h>
#include <U8g2lib.h>
#include <math.h>
#include "Display_Config.h"

// Cache variables for optimized rendering
struct {
    int last_frames = -1;
    float last_fps = -1;
    float last_humidity = -1;
    float last_temp = -999;
    uint8_t last_sprite = 255;
    char last_time_str[6] = "";      // HH:MM
    char last_date_str[11] = "";     // DD/MM/YYYY
    float last_gx = -999;
    float last_gy = -999;
} display_cache;

// Calculate total G-force magnitude (for display)
float getGForceMagnitude(int16_t x, int16_t y, int16_t z) {
    // ADXL345: 256 LSB/G at ±4G range
    float gx = x / 256.0f;
    float gy = y / 256.0f;
    float gz = z / 256.0f;
    return sqrt(gx*gx + gy*gy + gz*gz);
}

// Get individual X and Y G-force components
void getGForceComponents(int16_t x, int16_t y, float &gx_out, float &gy_out) {
    // ADXL345: 256 LSB/G at ±4G range
    gx_out = x / 256.0f;  // Roll component  
    gy_out = y / 256.0f;  // Pitch component (opposite sign, but we display absolute value)
    // Note: gy is negative when accelerating forward, positive when braking
    gy_out = -gy_out;
}

// Format time string for display
void formatTimeString(char *buffer, uint8_t hour, uint8_t minute) {
    sprintf(buffer, "%02d:%02d", hour, minute);
}

// Format date string for display
void formatDateString(char *buffer, uint8_t day, uint8_t month, uint16_t year) {
    sprintf(buffer, "%02d/%02d/%04d", day, month, year);
}

// Optimized display rendering with partial updates
void renderDisplay(U8G2_ST7920_128X64_F_HW_SPI &u8g2, 
                   int16_t accel_x, int16_t accel_y, int16_t accel_z,
                   float roll_filtered, float pitch_filtered,
                   int current_sprite,
                   int frames, float fps, float humidity, float temp,
                   uint8_t hour, uint8_t minute, uint8_t day, uint8_t month, uint16_t year) {
    
    u8g2.clearBuffer();

    // ========== TOP-RIGHT: DATE (JJ/MM/YYYY) ==========
    char date_str[11];
    formatDateString(date_str, day, month, year);
    bool date_changed = (strcmp(date_str, display_cache.last_date_str) != 0);
    
    if (date_changed) {
        strcpy(display_cache.last_date_str, date_str);
        u8g2.setCursor(82, 7);
        u8g2.print(date_str);
    }

    // ========== TOP-LEFT: DEBUG INFO ==========
    // Frame counter (changes every frame, so always redraw)
    u8g2.setCursor(0, 7);
    u8g2.print("Frame:");
    u8g2.setCursor(42, 7);
    u8g2.print(frames);

    // FPS (update 1x/sec)
    bool fps_changed = (fps != display_cache.last_fps);
    if (fps_changed) {
        display_cache.last_fps = fps;
        u8g2.setCursor(0, 15);
        u8g2.print(fps, 1);
        u8g2.setCursor(18, 15);
        u8g2.print("FPS");
    }

    // ========== MIDDLE-LEFT: TEMPERATURE & HUMIDITY ==========
    bool temp_changed = (temp != display_cache.last_temp);
    bool humidity_changed = (humidity != display_cache.last_humidity);
    
    if (temp_changed || humidity_changed) {
        display_cache.last_temp = temp;
        display_cache.last_humidity = humidity;
        
        u8g2.setCursor(0, 23);
        u8g2.print(humidity, 1);
        u8g2.setCursor(23, 23);
        u8g2.print("%H");
        
        u8g2.setCursor(0, 31);
        u8g2.print(temp, 1);
        u8g2.setCursor(23, 31);
        u8g2.print("\xB0""C");
    }

    // ========== BOTTOM-LEFT: TIME (HH:MM) ==========
    char time_str[6];
    formatTimeString(time_str, hour, minute);
    bool time_changed = (strcmp(time_str, display_cache.last_time_str) != 0);
    
    if (time_changed) {
        strcpy(display_cache.last_time_str, time_str);
        u8g2.setCursor(0, 63);
        u8g2.print(time_str);
    }

    // ========== CENTER: CAR SPRITE & G-FORCE INDICATORS ==========
    // Car sprite at (70, 10) with size 20x64
    u8g2.drawXBM(70, 10, 20, 64, bitmap_allArray[current_sprite]);

    // Get G-force values
    float gx = 0, gy = 0;
    getGForceComponents(accel_x, accel_y, gx, gy);
    
    bool gforce_changed = (gx != display_cache.last_gx) || (gy != display_cache.last_gy);
    display_cache.last_gx = gx;
    display_cache.last_gy = gy;

    if (gforce_changed) {
        // Format G-force values (keep 2 decimals, short format to fit)
        char gx_str[6], gy_str[6];
        
        // Format as "+X.X" or "-X.X" to save space
        if (gx >= 0) {
            snprintf(gx_str, sizeof(gx_str), "+%.1f", gx);
        } else {
            snprintf(gx_str, sizeof(gx_str), "%.1f", gx);
        }
        
        if (gy >= 0) {
            snprintf(gy_str, sizeof(gy_str), "+%.1f", gy);
        } else {
            snprintf(gy_str, sizeof(gy_str), "%.1f", gy);
        }

        // Display around car
        // Top (pitch)
        u8g2.setCursor(72, 8);
        u8g2.setFont(u8g2_font_tom_thumb_4x6_mf);
        u8g2.print(gy_str);
        u8g2.setFont(u8g2_font_profont10_mf); // Reset to normal font

        // Bottom (braking/acceleration)
        u8g2.setCursor(72, 78);
        u8g2.setFont(u8g2_font_tom_thumb_4x6_mf);
        u8g2.print(gy_str);
        u8g2.setFont(u8g2_font_profont10_mf);

        // Left (rolling left)
        u8g2.setCursor(60, 40);
        u8g2.setFont(u8g2_font_tom_thumb_4x6_mf);
        u8g2.print(gx_str);
        u8g2.setFont(u8g2_font_profont10_mf);

        // Right (rolling right)
        u8g2.setCursor(92, 40);
        u8g2.setFont(u8g2_font_tom_thumb_4x6_mf);
        u8g2.print(gx_str);
        u8g2.setFont(u8g2_font_profont10_mf);
    }

    // Send buffer to display
    u8g2.sendBuffer();
}

#endif
