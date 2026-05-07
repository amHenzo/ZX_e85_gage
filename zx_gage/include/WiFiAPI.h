#ifndef WIFI_API_H
#define WIFI_API_H

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// WiFi credentials for hotspot
const char* WIFI_SSID = "ZX_GAGE_SETUP";
const char* WIFI_PASS = "SetupZX2024";

AsyncWebServer wifi_server(80);
bool wifi_enabled = false;

// HTML page for time configuration
const char html_page[] PROGMEM = R"(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>ZX Gage - Configuration</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 800px;
            margin: 50px auto;
            padding: 20px;
            background-color: #f0f0f0;
        }
        .container {
            background: white;
            border-radius: 8px;
            padding: 30px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #333;
            text-align: center;
        }
        .info {
            background: #e3f2fd;
            border-left: 4px solid #2196F3;
            padding: 12px;
            margin-bottom: 20px;
            border-radius: 4px;
        }
        .form-group {
            margin-bottom: 15px;
        }
        label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
            color: #555;
        }
        input {
            width: 100%;
            padding: 10px;
            border: 1px solid #ddd;
            border-radius: 4px;
            box-sizing: border-box;
            font-size: 16px;
        }
        input:focus {
            outline: none;
            border-color: #2196F3;
            box-shadow: 0 0 5px #2196F3;
        }
        button {
            width: 100%;
            padding: 12px;
            background-color: #4CAF50;
            color: white;
            border: none;
            border-radius: 4px;
            font-size: 16px;
            font-weight: bold;
            cursor: pointer;
            margin-top: 10px;
        }
        button:hover {
            background-color: #45a049;
        }
        .status {
            margin-top: 20px;
            padding: 12px;
            border-radius: 4px;
            text-align: center;
            display: none;
        }
        .status.success {
            background-color: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
            display: block;
        }
        .status.error {
            background-color: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
            display: block;
        }
        .current-time {
            background: #f5f5f5;
            padding: 15px;
            border-radius: 4px;
            margin-bottom: 20px;
            text-align: center;
            font-size: 18px;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>⏰ ZX Gage - Configuration</h1>
        <div class="info">
            <strong>Hotspot:</strong> ZX_GAGE_SETUP | <strong>Password:</strong> SetupZX2024
        </div>
        
        <div class="current-time" id="currentTime">Current Time: --:--</div>
        
        <form id="timeForm">
            <div class="form-group">
                <label for="hour">Heure (00-23):</label>
                <input type="number" id="hour" name="hour" min="0" max="23" required>
            </div>
            <div class="form-group">
                <label for="minute">Minutes (00-59):</label>
                <input type="number" id="minute" name="minute" min="0" max="59" required>
            </div>
            <div class="form-group">
                <label for="day">Jour (01-31):</label>
                <input type="number" id="day" name="day" min="1" max="31" required>
            </div>
            <div class="form-group">
                <label for="month">Mois (01-12):</label>
                <input type="number" id="month" name="month" min="1" max="12" required>
            </div>
            <div class="form-group">
                <label for="year">Année (2024-2099):</label>
                <input type="number" id="year" name="year" min="2024" max="2099" required>
            </div>
            <button type="submit">💾 Enregistrer l'heure</button>
        </form>
        
        <div class="status" id="status"></div>
    </div>

    <script>
        // Fetch and update current time
        async function updateTime() {
            try {
                const response = await fetch('/api/time');
                const data = await response.json();
                document.getElementById('currentTime').textContent = 
                    `Current Time: ${String(data.hour).padStart(2, '0')}:${String(data.minute).padStart(2, '0')} - ${String(data.day).padStart(2, '0')}/${String(data.month).padStart(2, '0')}/${data.year}`;
                
                // Pre-fill form with current time
                document.getElementById('hour').value = String(data.hour).padStart(2, '0');
                document.getElementById('minute').value = String(data.minute).padStart(2, '0');
                document.getElementById('day').value = String(data.day).padStart(2, '0');
                document.getElementById('month').value = String(data.month).padStart(2, '0');
                document.getElementById('year').value = data.year;
            } catch (error) {
                console.error('Error fetching time:', error);
            }
        }

        // Submit form
        document.getElementById('timeForm').addEventListener('submit', async (e) => {
            e.preventDefault();
            
            const timeData = {
                hour: parseInt(document.getElementById('hour').value),
                minute: parseInt(document.getElementById('minute').value),
                second: 0,
                day: parseInt(document.getElementById('day').value),
                month: parseInt(document.getElementById('month').value),
                year: parseInt(document.getElementById('year').value)
            };

            try {
                const response = await fetch('/api/time', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify(timeData)
                });

                const result = await response.json();
                const statusDiv = document.getElementById('status');
                
                if (response.ok) {
                    statusDiv.className = 'status success';
                    statusDiv.textContent = '✅ ' + result.message;
                    setTimeout(() => {
                        statusDiv.className = 'status';
                    }, 3000);
                    updateTime();
                } else {
                    statusDiv.className = 'status error';
                    statusDiv.textContent = '❌ ' + result.error;
                }
            } catch (error) {
                const statusDiv = document.getElementById('status');
                statusDiv.className = 'status error';
                statusDiv.textContent = '❌ Erreur de communication: ' + error;
            }
        });

        // Initial update and refresh every 5 seconds
        updateTime();
        setInterval(updateTime, 5000);
    </script>
</body>
</html>
)";

// Initialize WiFi hotspot
void initWiFi() {
    // Stop any existing WiFi
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    
    IPAddress localIP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    
    WiFi.softAPConfig(localIP, gateway, subnet);
    
    Serial.println("[WiFi] Hotspot started");
    Serial.printf("[WiFi] SSID: %s\n", WIFI_SSID);
    Serial.printf("[WiFi] Password: %s\n", WIFI_PASS);
    Serial.printf("[WiFi] IP: 192.168.4.1\n");

    wifi_enabled = true;
}

// Setup WiFi API endpoints - needs RTC functions
void setupWiFiAPI() {
    // GET /api/time - return current time/date
    wifi_server.on("/api/time", HTTP_GET, [](AsyncWebServerRequest *request) {
        DynamicJsonDocument doc(256);
        doc["hour"] = rtc_hour;
        doc["minute"] = rtc_minute;
        doc["second"] = rtc_second;
        doc["day"] = rtc_day;
        doc["month"] = rtc_month;
        doc["year"] = rtc_year;
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    // POST /api/time - set new time/date
    wifi_server.on("/api/time", HTTP_POST, [](AsyncWebServerRequest *request) {
        // Handled in body handler
    }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        DynamicJsonDocument doc(256);
        DeserializationError error = deserializeJson(doc, data);
        
        if (error) {
            DynamicJsonDocument errDoc(128);
            errDoc["error"] = "Invalid JSON";
            String response;
            serializeJson(errDoc, response);
            request->send(400, "application/json", response);
            return;
        }

        uint8_t hour = doc["hour"] | rtc_hour;
        uint8_t minute = doc["minute"] | rtc_minute;
        uint8_t second = doc["second"] | 0;
        uint8_t day = doc["day"] | rtc_day;
        uint8_t month = doc["month"] | rtc_month;
        uint16_t year = doc["year"] | rtc_year;

        // Validate
        if (hour > 23 || minute > 59 || second > 59 || day > 31 || month > 12) {
            DynamicJsonDocument errDoc(128);
            errDoc["error"] = "Invalid time values";
            String response;
            serializeJson(errDoc, response);
            request->send(400, "application/json", response);
            return;
        }

        // Set time
        setRTC(hour, minute, second, day, month, year);

        DynamicJsonDocument respDoc(256);
        respDoc["message"] = "Time updated successfully";
        respDoc["hour"] = hour;
        respDoc["minute"] = minute;
        respDoc["day"] = day;
        respDoc["month"] = month;
        respDoc["year"] = year;
        
        String response;
        serializeJson(respDoc, response);
        request->send(200, "application/json", response);
    });

    // Serve HTML page at root
    wifi_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", html_page);
    });

    // Start server
    wifi_server.begin();
    Serial.println("[WiFi] Web server started on port 80");
}

#endif
