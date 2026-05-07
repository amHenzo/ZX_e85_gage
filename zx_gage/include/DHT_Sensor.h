#ifndef DHT_SENSOR_H
#define DHT_SENSOR_H

#include <Arduino.h>
#include "DHT.h"

// DHT11 Configuration
#define DHT_PIN 17
#define DHT_TYPE DHT11

// DHT instance
DHT dht(DHT_PIN, DHT_TYPE);

// Current sensor readings
float current_humidity = 0.0f;
float current_temperature = 0.0f;

// Last read time
unsigned long last_dht_read = 0;
const unsigned long DHT_READ_INTERVAL = 2000; // Read every 2 seconds (DHT11 is slow)

// Initialize DHT sensor
void initDHTSensor() {
    dht.begin();
    Serial.println("[DHT11] Temperature/Humidity sensor initialized");
}

// Read temperature and humidity from DHT11
void readDHTSensor() {
    // Throttle DHT reads (DHT11 is slow: ~2sec response time)
    if (millis() - last_dht_read < DHT_READ_INTERVAL) {
        return;
    }
    last_dht_read = millis();
    
    // Read humidity and temperature
    float humidity_raw = dht.readHumidity();
    float temp_raw = dht.readTemperature();
    
    // Check if reads are valid (DHT returns NaN on error)
    if (!isnan(humidity_raw)) {
        current_humidity = humidity_raw;
    }
    
    if (!isnan(temp_raw)) {
        current_temperature = temp_raw;
    }
    
    Serial.printf("[DHT11] Temp: %.1f°C | Humidity: %.1f%%\n", current_temperature, current_humidity);
}

// Get current temperature
float getTherature() {
    return current_temperature;
}

// Get current humidity
float getHumidity() {
    return current_humidity;
}

#endif
