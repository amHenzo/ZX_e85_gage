#include "web_portal.h"

#include <Arduino.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_state.h"
#include "generated_web_assets.h"
#include "time_service.h"

namespace {
constexpr char AP_SSID[] = "ZX-Gage-Setup";
constexpr uint16_t DNS_PORT = 53;

AsyncWebServer server(80);
AsyncEventSource events("/events");
DNSServer dnsServer;

bool readPostTime(AsyncWebServerRequest *request, ManualTime &time)
{
  const char *names[] = {"year", "month", "day", "hour", "minute", "second"};
  for (const char *name : names) {
    if (!request->hasParam(name, true)) {
      return false;
    }
  }

  time.year = request->getParam("year", true)->value().toInt();
  time.month = request->getParam("month", true)->value().toInt();
  time.day = request->getParam("day", true)->value().toInt();
  time.hour = request->getParam("hour", true)->value().toInt();
  time.minute = request->getParam("minute", true)->value().toInt();
  time.second = request->getParam("second", true)->value().toInt();
  return true;
}

void sendPortalPage(AsyncWebServerRequest *request)
{
  request->send_P(200, "text/html", WEB_INDEX_HTML);
}

void dnsTask(void *)
{
  for (;;) {
    dnsServer.processNextRequest();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void streamTask(void *)
{
  for (;;) {
    ZxGaugeState snapshot;
    appStateGet(snapshot);

    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"temp\":%.2f,\"hum\":%.2f,\"fps\":%.2f}",
             snapshot.accelXG, snapshot.accelYG, snapshot.accelZG, snapshot.temperatureC,
             snapshot.humidityPct, snapshot.fps);
    events.send(payload, "state", millis());
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
}

void webPortalStart()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, sendPortalPage);
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/css", WEB_STYLE_CSS);
  });
  server.on("/app.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "application/javascript", WEB_APP_JS);
  });
  server.on("/generate_204", HTTP_GET, sendPortalPage);
  server.on("/gen_204", HTTP_GET, sendPortalPage);
  server.on("/hotspot-detect.html", HTTP_GET, sendPortalPage);
  server.on("/library/test/success.html", HTTP_GET, sendPortalPage);
  server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->redirect("/");
  });
  server.on("/ncsi.txt", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->redirect("/");
  });
  server.addHandler(&events);

  server.on("/time", HTTP_POST, [](AsyncWebServerRequest *request) {
    ManualTime manualTime;
    if (!readPostTime(request, manualTime) || !timeServiceSetManual(manualTime)) {
      request->send(400, "text/plain", "Invalid time value");
      return;
    }

    request->send(200, "text/html", "<!doctype html><html><body><h1>Time saved</h1><p>DS3231 updated.</p><a href=\"/\">Back</a></body></html>");
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect("/");
  });

  server.begin();
  xTaskCreatePinnedToCore(dnsTask, "portal-dns", 2048, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(streamTask, "portal-events", 4096, nullptr, 1, nullptr, 0);
  appStateSetPortalReady(true);
}
