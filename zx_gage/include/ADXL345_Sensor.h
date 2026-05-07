#ifndef ADXL345_SENSOR_H
#define ADXL345_SENSOR_H

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// ADXL345 Chip Select
#define CS_ADXL 16

// ADXL345 Register addresses
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32
#define DATAX1 0x33
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x36
#define DATAZ1 0x37

// Raw accelerometer values
int16_t accel_x = 0;
int16_t accel_y = 0;
int16_t accel_z = 0;

// Calculated angles
float roll = 0.0f;
float pitch = 0.0f;

// Filtered angles (IIR low-pass)
float roll_filtered = 0.0f;
float pitch_filtered = 0.0f;

// Buffer for SPI reads
char spi_buffer[10];

// Write register to ADXL345
void adxl_write_register(uint8_t reg_addr, uint8_t value) {
    digitalWrite(CS_ADXL, LOW);
    SPI.transfer(reg_addr);
    SPI.transfer(value);
    digitalWrite(CS_ADXL, HIGH);
}

// Read registers from ADXL345
void adxl_read_registers(uint8_t reg_addr, uint8_t num_bytes, uint8_t *buffer) {
    uint8_t address = 0x80 | reg_addr;
    if (num_bytes > 1) {
        address |= 0x40;
    }
    
    digitalWrite(CS_ADXL, LOW);
    SPI.transfer(address);
    for (int i = 0; i < num_bytes; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(CS_ADXL, HIGH);
}

// Initialize ADXL345
void initADXL345() {
    SPI.begin();
    SPI.setDataMode(SPI_MODE3);
    SPI.setClockDivider(SPI_CLOCK_DIV4);
    
    pinMode(CS_ADXL, OUTPUT);
    digitalWrite(CS_ADXL, HIGH);
    
    // Set to ±4G range
    adxl_write_register(DATA_FORMAT, 0x01);
    
    // Enable measurement mode
    adxl_write_register(POWER_CTL, 0x08);
    
    Serial.println("[ADXL345] Accelerometer initialized");
}

// Read raw accelerometer data and calculate angles
void readADXL345() {
    adxl_read_registers(DATAX0, 6, (uint8_t*)spi_buffer);
    
    // Combine bytes for 16-bit values
    accel_x = ((int16_t)spi_buffer[1] << 8) | (int16_t)spi_buffer[0];
    accel_y = ((int16_t)spi_buffer[3] << 8) | (int16_t)spi_buffer[2];
    accel_z = ((int16_t)spi_buffer[5] << 8) | (int16_t)spi_buffer[4];
    
    // Calculate pitch and roll angles (in degrees)
    roll = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * 180.0f / PI;
    pitch = atan(-1.0f * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * 180.0f / PI;
    
    // Apply low-pass IIR filter (α = 0.06, smooth over ~16 frames)
    roll_filtered = 0.94f * roll_filtered + 0.06f * roll;
    pitch_filtered = 0.94f * pitch_filtered + 0.06f * pitch;
    
    // Debug output to serial
    Serial.print(accel_x, DEC);
    Serial.print(',');
    Serial.print(accel_y, DEC);
    Serial.print(',');
    Serial.print(accel_z, DEC);
    Serial.print(',');
    Serial.print(roll, 2);
    Serial.print(',');
    Serial.println(pitch, 2);
}

// Determine sprite based on filtered angles
int getSpriteFromAngles(float pitch_ang, float roll_ang) {
    if (abs(pitch_ang) < 15 && abs(roll_ang) < 15) {
        return 2; // Normal
    }
    
    if (abs(pitch_ang) > abs(roll_ang)) {
        if (pitch_ang < 0) {
            return 1; // Left
        } else {
            return 3; // Right
        }
    } else {
        if (roll_ang < 0) {
            return 0; // Braking
        } else {
            return 4; // Speeding
        }
    }
}

#endif
