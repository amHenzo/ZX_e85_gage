#include "web_portal.h"

#include <Arduino.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_state.h"
#include "time_service.h"

namespace {
constexpr char AP_SSID[] = "ZX-Gage-Setup";
constexpr uint16_t DNS_PORT = 53;

AsyncWebServer server(80);
DNSServer dnsServer;

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ZX Gage Setup</title>
  <style>
    body{font-family:system-ui,Arial,sans-serif;margin:24px;background:#101214;color:#f5f5f5}
    main{max-width:420px;margin:auto}
    label{display:block;margin:12px 0 4px}
    input,button{box-sizing:border-box;width:100%;font-size:18px;padding:10px;border-radius:6px;border:1px solid #555}
    button{margin-top:18px;background:#f5f5f5;color:#101214;font-weight:700}
    .row{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
  </style>
</head>
<body>
<main>
  <h1>ZX Gage Time</h1>
  <form method="post" action="/time">
    <label>Date</label>
    <div class="row">
      <input name="year" type="number" min="2020" max="2099" placeholder="YYYY" required>
      <input name="month" type="number" min="1" max="12" placeholder="MM" required>
      <input name="day" type="number" min="1" max="31" placeholder="DD" required>
    </div>
    <label>Time</label>
    <div class="row">
      <input name="hour" type="number" min="0" max="23" placeholder="HH" required>
      <input name="minute" type="number" min="0" max="59" placeholder="MM" required>
      <input name="second" type="number" min="0" max="59" placeholder="SS" value="0" required>
    </div>
    <button type="submit">Set DS3231 Time</button>
  </form>
</main>
</body>
</html>
)HTML";

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
  request->send_P(200, "text/html", INDEX_HTML);
}

void dnsTask(void *)
{
  for (;;) {
    dnsServer.processNextRequest();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
}

void webPortalStart()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, sendPortalPage);
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
  appStateSetPortalReady(true);
}
