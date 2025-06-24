#include <Arduino.h>
#include <Preferences.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>

const int reedPin = 18;
const int pwmPin = 19;
const int pwmChannel = 0;

Preferences prefs;
AsyncWebServer server(80);

volatile unsigned long lastPulse = 0;
volatile unsigned long pulseInterval = 1000;

float pulsesPerRev = 1.0; // saved in NVS
float speed = 0.0;

void IRAM_ATTR onPulse() {
  unsigned long now = micros();
  pulseInterval = now - lastPulse;
  lastPulse = now;
}

void speedTask(void *param) {
  for (;;) {
    if (pulseInterval > 0) {
      float freq = 1000000.0 / pulseInterval;
      speed = freq / pulsesPerRev;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void pwmTask(void *param) {
  for (;;) {
    int duty = constrain((int)speed, 0, 255);
    ledcWrite(pwmChannel, duty);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setupOTA() {
  ArduinoOTA
    .onStart([]() { Serial.println("OTA Start"); })
    .onEnd([]() { Serial.println("OTA End"); })
    .onError([](ota_error_t error) { Serial.printf("OTA Error %u\n", error); });
  ArduinoOTA.begin();
}

void setupWeb() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(SPIFFS, "/dashboard.html", "text/html");
  });
  server.on("/metrics", HTTP_GET, [](AsyncWebServerRequest *req){
    DynamicJsonDocument doc(256);
    doc["speed"] = speed;
    doc["pulsesPerRev"] = pulsesPerRev;
    String out;
    serializeJson(doc, out);
    req->send(200, "application/json", out);
  });
  server.on("/calibrate", HTTP_POST, [](AsyncWebServerRequest *req){
    if(req->hasParam("body", true)) {
      DynamicJsonDocument doc(128);
      deserializeJson(doc, req->getParam("body", true)->value());
      pulsesPerRev = doc["pulses"].as<float>();
      prefs.putFloat("pulses", pulsesPerRev);
      req->send(200, "text/plain", "ok");
    } else {
      req->send(400, "text/plain", "bad request");
    }
  });
  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *req){}, NULL, [](){ ArduinoOTA.handle(); });
  server.begin();
}

void setup() {
  Serial.begin(115200);
  prefs.begin("speed", false);
  pulsesPerRev = prefs.getFloat("pulses", 1.0);

  pinMode(reedPin, INPUT_PULLUP);
  attachInterrupt(reedPin, onPulse, FALLING);

  ledcSetup(pwmChannel, 5000, 8);
  ledcAttachPin(pwmPin, pwmChannel);

  if(!SPIFFS.begin(true))
    Serial.println("SPIFFS failed");

  xTaskCreatePinnedToCore(speedTask, "speed", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(pwmTask, "pwm", 2048, NULL, 1, NULL, 1);

  setupWeb();
  setupOTA();
}

void loop() {
  ArduinoOTA.handle();
}

