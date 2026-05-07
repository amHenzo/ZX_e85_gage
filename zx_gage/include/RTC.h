#ifndef RTC_H
#define RTC_H

#include <Wire.h>
#include <Arduino.h>

// DS3231 I2C address
#define DS3231_ADDR 0x68

// DS3231 Register addresses
#define DS3231_SEC_REG 0x00
#define DS3231_MIN_REG 0x01
#define DS3231_HOUR_REG 0x02
#define DS3231_DATE_REG 0x04
#define DS3231_MONTH_REG 0x05
#define DS3231_YEAR_REG 0x06

// Global RTC variables
uint8_t rtc_hour = 0;
uint8_t rtc_minute = 0;
uint8_t rtc_second = 0;
uint8_t rtc_day = 1;
uint8_t rtc_month = 1;
uint16_t rtc_year = 2024;

unsigned long last_rtc_read = 0;

// Convert BCD to decimal
uint8_t bcd_to_decimal(uint8_t bcd) {
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

// Convert decimal to BCD
uint8_t decimal_to_bcd(uint8_t decimal) {
    return ((decimal / 10) << 4) | (decimal % 10);
}

// Initialize RTC
void initRTC() {
    Wire.begin(21, 4); // SDA=GPIO21, SCL=GPIO4
    Wire.setClock(100000); // 100kHz I2C speed
    Serial.println("[RTC] DS3231 initialized on I2C");
}

// Read current time from DS3231
void readRTC() {
    // Limit reads to 1 per second to reduce I2C traffic
    if (millis() - last_rtc_read < 1000) {
        return;
    }
    last_rtc_read = millis();

    Wire.beginTransmission(DS3231_ADDR);
    Wire.write(DS3231_SEC_REG); // Start at seconds register
    Wire.endTransmission();

    // Read 7 bytes: SEC, MIN, HOUR, DAY, DATE, MONTH, YEAR
    Wire.requestFrom(DS3231_ADDR, 7, true);

    if (Wire.available() >= 7) {
        uint8_t sec_raw = Wire.read();
        uint8_t min_raw = Wire.read();
        uint8_t hour_raw = Wire.read();
        Wire.read(); // DAY of week - ignore
        uint8_t date_raw = Wire.read();
        uint8_t month_raw = Wire.read();
        uint8_t year_raw = Wire.read();

        rtc_second = bcd_to_decimal(sec_raw & 0x7F);
        rtc_minute = bcd_to_decimal(min_raw & 0x7F);
        rtc_hour = bcd_to_decimal(hour_raw & 0x3F); // 24-hour format
        rtc_day = bcd_to_decimal(date_raw & 0x3F);
        rtc_month = bcd_to_decimal(month_raw & 0x1F);
        rtc_year = 2000 + bcd_to_decimal(year_raw);
    }
}

// Set time on DS3231
void setRTC(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day, uint8_t month, uint16_t year) {
    Wire.beginTransmission(DS3231_ADDR);
    Wire.write(DS3231_SEC_REG);
    Wire.write(decimal_to_bcd(second));
    Wire.write(decimal_to_bcd(minute));
    Wire.write(decimal_to_bcd(hour));
    Wire.write(0x01); // Day of week (dummy)
    Wire.write(decimal_to_bcd(day));
    Wire.write(decimal_to_bcd(month));
    Wire.write(decimal_to_bcd(year - 2000));
    Wire.endTransmission();

    rtc_hour = hour;
    rtc_minute = minute;
    rtc_second = second;
    rtc_day = day;
    rtc_month = month;
    rtc_year = year;

    Serial.printf("[RTC] Time set to %02d:%02d:%02d %02d/%02d/%04d\n", 
                  hour, minute, second, day, month, year);
}

#endif
