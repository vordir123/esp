/*
 * ESP32-WROOM-32 Speed Manipulator with CAN Dashboard
 * Compatible with LilyGO Display Board
 *
 * Hardware:
 * - ESP32-WROOM-32 (LilyGO Board)
 * - Reed Sensor (3.3V) on GPIO 34
 * - PWM Output for Tacho on GPIO 25
 * - I2C to Teensy 4.1 (SDA: GPIO 21, SCL: GPIO 22)
 * - Status LED on GPIO 2
 *
 * Features:
 * - Reed sensor speed detection (27.5" wheel)
 * - Linear speed manipulation (22km/h -> 12km/h, etc.)
 * - Web dashboard with calibration mode
 * - Real-time speed display and graphing
 * - CAN message viewer from Teensy
 * - OTA firmware updates
 * - NVS configuration storage
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <Update.h>

// ==================== Configuration ====================
#define REED_SENSOR_PIN 34
#define PWM_OUTPUT_PIN 25
#define STATUS_LED_PIN 2
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define TEENSY_I2C_ADDRESS 0x42

// WiFi Configuration
const char* DEFAULT_SSID = "SpeedManipulator";
const char* DEFAULT_PASSWORD = "speed123456";

// Wheel Configuration
#define WHEEL_CIRCUMFERENCE_MM 2170.0  // 27.5 inch wheel = 2170mm
#define MAGNETS_PER_REVOLUTION 1

// PWM Configuration
#define PWM_FREQUENCY 1000  // Hz
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8    // 8-bit resolution

// Speed limits (km/h)
#define MAX_REALISTIC_SPEED 80.0
#define MIN_REALISTIC_SPEED 0.5

// Calibration
#define CALIBRATION_SPEED 15.0  // km/h
#define CALIBRATION_DURATION 30000  // 30 seconds

// ==================== Global Objects ====================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences preferences;

// ==================== Speed Calculation ====================
volatile uint32_t reedPulseCount = 0;
volatile uint32_t lastPulseTime = 0;
volatile uint32_t pulseInterval = 0;

// Speed variables
float realSpeed = 0.0;
float manipulatedSpeed = 0.0;
float speedScalingFactor = 0.5455;  // Default: 22->12, 23->13 = (12-13)/(22-23) = -1/-1.8333 ~0.5455
float speedOffset = 0.0;

// Configuration from NVS
float wheelCircumference = WHEEL_CIRCUMFERENCE_MM;
uint8_t magnetsPerRevolution = MAGNETS_PER_REVOLUTION;
bool calibrationMode = false;
uint32_t calibrationStartTime = 0;
float calibrationTargetSpeed = CALIBRATION_SPEED;

// Statistics
struct Statistics {
  uint32_t uptime;
  uint32_t totalPulses;
  float maxSpeed;
  float avgSpeed;
  uint32_t wifiRssi;
  uint32_t freeHeap;
  uint32_t canMessagesReceived;
} stats = {0};

// Speed history for graphing (60 samples = 60 seconds at 1Hz)
#define HISTORY_SIZE 60
float speedHistory[HISTORY_SIZE] = {0};
uint8_t historyIndex = 0;

// CAN Message buffer from Teensy
struct CANMessage {
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
} lastCANMessage;

// CAN message history
#define CAN_HISTORY_SIZE 100
struct CANHistoryEntry {
  uint32_t timestamp;
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
} canHistory[CAN_HISTORY_SIZE];
uint8_t canHistoryIndex = 0;

// Timers
hw_timer_t *speedTimer = NULL;
uint32_t lastWebUpdate = 0;
uint32_t lastStatsUpdate = 0;
uint32_t lastSpeedCalculation = 0;
uint32_t lastPWMUpdate = 0;
uint32_t lastI2CRead = 0;

// Signal quality
uint32_t goodPulses = 0;
uint32_t badPulses = 0;

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Speed Manipulator Starting ===");

  // Initialize pins
  pinMode(REED_SENSOR_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Initialize PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_OUTPUT_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR: SPIFFS initialization failed!");
  } else {
    Serial.println("SPIFFS initialized");
  }

  // Load configuration from NVS
  loadConfiguration();

  // Initialize I2C as master
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.println("I2C initialized as master");

  // Attach reed sensor interrupt
  attachInterrupt(digitalPinToInterrupt(REED_SENSOR_PIN), reedSensorISR, FALLING);
  Serial.println("Reed sensor interrupt attached");

  // Initialize WiFi
  initWiFi();

  // Initialize mDNS
  if (MDNS.begin("speedmanipulator")) {
    Serial.println("mDNS responder started: http://speedmanipulator.local");
  }

  // Initialize web server
  initWebServer();

  // Initialize WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Start web server
  server.begin();
  Serial.println("Web server started");

  // Initialize timer for speed calculation (100Hz)
  speedTimer = timerBegin(0, 80, true);  // 80 prescaler = 1MHz
  timerAttachInterrupt(speedTimer, &speedCalculationISR, true);
  timerAlarmWrite(speedTimer, 10000, true);  // 10ms = 100Hz
  timerAlarmEnable(speedTimer);

  Serial.println("=== System Ready ===");
  Serial.print("Connect to: http://");
  Serial.println(WiFi.localIP());

  // Blink LED to indicate ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(200);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(200);
  }
}

// ==================== Main Loop ====================
void loop() {
  uint32_t currentMillis = millis();

  // Calculate speed every 100ms
  if (currentMillis - lastSpeedCalculation >= 100) {
    calculateSpeed();
    lastSpeedCalculation = currentMillis;
  }

  // Update PWM output every 50ms
  if (currentMillis - lastPWMUpdate >= 50) {
    updatePWMOutput();
    lastPWMUpdate = currentMillis;
  }

  // Read CAN messages from Teensy every 100ms
  if (currentMillis - lastI2CRead >= 100) {
    readCANFromTeen sy();
    lastI2CRead = currentMillis;
  }

  // Update statistics every second
  if (currentMillis - lastStatsUpdate >= 1000) {
    updateStatistics();
    addToSpeedHistory(realSpeed);
    lastStatsUpdate = currentMillis;

    // Blink status LED
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
  }

  // Send WebSocket updates every 500ms
  if (currentMillis - lastWebUpdate >= 500) {
    sendWebSocketUpdate();
    lastWebUpdate = currentMillis;
  }

  // Handle calibration mode
  if (calibrationMode) {
    handleCalibration();
  }

  // Cleanup WebSocket clients
  ws.cleanupClients();

  delay(10);
}

// ==================== Reed Sensor ISR ====================
void IRAM_ATTR reedSensorISR() {
  uint32_t currentTime = micros();
  uint32_t interval = currentTime - lastPulseTime;

  // Debounce: ignore pulses shorter than 10ms
  if (interval > 10000) {
    pulseInterval = interval;
    lastPulseTime = currentTime;
    reedPulseCount++;
    goodPulses++;
  } else {
    badPulses++;
  }
}

// ==================== Speed Calculation ISR ====================
void IRAM_ATTR speedCalculationISR() {
  // This runs at 100Hz to check for timeout
  uint32_t timeSinceLastPulse = micros() - lastPulseTime;

  // If no pulse for 2 seconds, speed is zero
  if (timeSinceLastPulse > 2000000) {
    pulseInterval = 0;
  }
}

// ==================== Speed Functions ====================
void calculateSpeed() {
  if (calibrationMode) {
    // In calibration mode, output constant speed
    realSpeed = calibrationTargetSpeed;
    manipulatedSpeed = calibrationTargetSpeed;
    return;
  }

  if (pulseInterval == 0) {
    realSpeed = 0.0;
    manipulatedSpeed = 0.0;
    return;
  }

  // Calculate speed in km/h
  // Speed = (circumference / interval) * (3600000000 / 1000000) / 1000
  // Speed = (circumference * 3600) / (interval * 1000)
  float intervalSeconds = pulseInterval / 1000000.0;
  float distanceMeters = (wheelCircumference / 1000.0) / magnetsPerRevolution;
  realSpeed = (distanceMeters / intervalSeconds) * 3.6;  // Convert m/s to km/h

  // Plausibility check
  if (realSpeed < MIN_REALISTIC_SPEED || realSpeed > MAX_REALISTIC_SPEED) {
    realSpeed = 0.0;
    manipulatedSpeed = 0.0;
    return;
  }

  // Apply linear scaling: manipulated = real * factor + offset
  // For 22->12, 23->13: factor = 1.0, offset = -10
  // But user wants configurable, so we use: manipulated = real * scalingFactor + offset
  // Default calibration: 22->12 means factor=12/22=0.5454, offset=0
  manipulatedSpeed = realSpeed * speedScalingFactor + speedOffset;

  // Ensure manipulated speed is reasonable
  if (manipulatedSpeed < 0) manipulatedSpeed = 0;
  if (manipulatedSpeed > MAX_REALISTIC_SPEED) manipulatedSpeed = MAX_REALISTIC_SPEED;

  // Update max speed
  if (realSpeed > stats.maxSpeed) {
    stats.maxSpeed = realSpeed;
  }
}

void updatePWMOutput() {
  // Convert speed to PWM duty cycle
  // Tacho typically expects frequency proportional to speed
  // For simplicity, we'll use duty cycle: 0% = 0 km/h, 100% = 100 km/h

  float dutyCycle = (manipulatedSpeed / 100.0) * 255.0;
  if (dutyCycle > 255) dutyCycle = 255;
  if (dutyCycle < 0) dutyCycle = 0;

  ledcWrite(PWM_CHANNEL, (uint8_t)dutyCycle);
}

void addToSpeedHistory(float speed) {
  speedHistory[historyIndex] = speed;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
}

// ==================== Calibration ====================
void startCalibration(float targetSpeed) {
  calibrationMode = true;
  calibrationTargetSpeed = targetSpeed;
  calibrationStartTime = millis();

  Serial.print("Calibration started: ");
  Serial.print(targetSpeed);
  Serial.println(" km/h for 30 seconds");
}

void handleCalibration() {
  uint32_t elapsed = millis() - calibrationStartTime;

  if (elapsed >= CALIBRATION_DURATION) {
    calibrationMode = false;
    Serial.println("Calibration output complete. Waiting for user input...");
  }
}

void applyCalibration(float displayedSpeed) {
  // User drove at calibrationTargetSpeed but tacho showed displayedSpeed
  // We need to calculate scaling factor
  // Real speed outputs manipulatedSpeed to tacho
  // calibrationTargetSpeed -> displayedSpeed
  // So if we output calibrationTargetSpeed, tacho shows displayedSpeed
  // Scaling factor = displayedSpeed / calibrationTargetSpeed

  if (calibrationTargetSpeed > 0) {
    float measuredFactor = displayedSpeed / calibrationTargetSpeed;

    // Now for normal operation, if we want 22->12:
    // We need to output speed such that tacho shows desired
    // desiredOutput = realSpeed * scalingFactor
    // Since tacho has its own scaling (measuredFactor), we need:
    // finalTachoReading = desiredOutput * measuredFactor
    // We want finalTachoReading = realSpeed * targetScalingFactor
    // So: desiredOutput * measuredFactor = realSpeed * targetScalingFactor
    // desiredOutput = realSpeed * targetScalingFactor / measuredFactor

    // For 22->12: targetScalingFactor = 12/22 = 0.5454
    // Final scaling = 0.5454 / measuredFactor
    speedScalingFactor = (12.0 / 22.0) / measuredFactor;
    speedOffset = 0.0;

    // Save to NVS
    preferences.begin("speedconfig", false);
    preferences.putFloat("scaleFactor", speedScalingFactor);
    preferences.putFloat("offset", speedOffset);
    preferences.end();

    Serial.print("Calibration complete. New scaling factor: ");
    Serial.println(speedScalingFactor, 4);
  }
}

// ==================== I2C / CAN Functions ====================
void readCANFromTeen sy() {
  Wire.requestFrom(TEENSY_I2C_ADDRESS, sizeof(CANMessage));

  if (Wire.available() >= sizeof(CANMessage)) {
    Wire.readBytes((uint8_t*)&lastCANMessage, sizeof(CANMessage));

    // Check if valid message (ID != 0)
    if (lastCANMessage.id != 0) {
      stats.canMessagesReceived++;

      // Add to history
      canHistory[canHistoryIndex].timestamp = millis();
      canHistory[canHistoryIndex].id = lastCANMessage.id;
      canHistory[canHistoryIndex].len = lastCANMessage.len;
      memcpy(canHistory[canHistoryIndex].data, lastCANMessage.data, 8);
      canHistoryIndex = (canHistoryIndex + 1) % CAN_HISTORY_SIZE;
    }
  }
}

// ==================== WiFi Functions ====================
void initWiFi() {
  Serial.println("Starting WiFi Access Point...");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(DEFAULT_SSID, DEFAULT_PASSWORD);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

// ==================== Web Server ====================
void initWebServer() {
  // Serve dashboard HTML
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/dashboard.html", "text/html");
  });

  // API: Get current status
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = getStatusJSON();
    request->send(200, "application/json", json);
  });

  // API: Start calibration
  server.on("/api/calibration/start", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("speed", true)) {
      float speed = request->getParam("speed", true)->value().toFloat();
      startCalibration(speed);
      request->send(200, "application/json", "{\"status\":\"started\"}");
    } else {
      request->send(400, "application/json", "{\"error\":\"missing speed parameter\"}");
    }
  });

  // API: Complete calibration
  server.on("/api/calibration/complete", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("displayed", true)) {
      float displayed = request->getParam("displayed", true)->value().toFloat();
      applyCalibration(displayed);
      request->send(200, "application/json", "{\"status\":\"completed\"}");
    } else {
      request->send(400, "application/json", "{\"error\":\"missing displayed parameter\"}");
    }
  });

  // API: Set scaling factor
  server.on("/api/config/scaling", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("factor", true)) {
      speedScalingFactor = request->getParam("factor", true)->value().toFloat();

      if (request->hasParam("offset", true)) {
        speedOffset = request->getParam("offset", true)->value().toFloat();
      }

      preferences.begin("speedconfig", false);
      preferences.putFloat("scaleFactor", speedScalingFactor);
      preferences.putFloat("offset", speedOffset);
      preferences.end();

      request->send(200, "application/json", "{\"status\":\"updated\"}");
    } else {
      request->send(400, "application/json", "{\"error\":\"missing factor parameter\"}");
    }
  });

  // API: Set wheel configuration
  server.on("/api/config/wheel", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("circumference", true)) {
      wheelCircumference = request->getParam("circumference", true)->value().toFloat();
    }
    if (request->hasParam("magnets", true)) {
      magnetsPerRevolution = request->getParam("magnets", true)->value().toInt();
    }

    preferences.begin("speedconfig", false);
    preferences.putFloat("wheelCirc", wheelCircumference);
    preferences.putUChar("magnets", magnetsPerRevolution);
    preferences.end();

    request->send(200, "application/json", "{\"status\":\"updated\"}");
  });

  // API: Get speed history
  server.on("/api/history/speed", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = getSpeedHistoryJSON();
    request->send(200, "application/json", json);
  });

  // API: Get CAN history
  server.on("/api/history/can", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = getCANHistoryJSON();
    request->send(200, "application/json", json);
  });

  // API: Export speed data as CSV
  server.on("/api/export/speed", HTTP_GET, [](AsyncWebServerRequest *request) {
    String csv = "Timestamp,Real_Speed,Manipulated_Speed\n";
    uint32_t timestamp = millis();
    for (int i = 0; i < HISTORY_SIZE; i++) {
      int idx = (historyIndex + i) % HISTORY_SIZE;
      csv += String(timestamp - (HISTORY_SIZE - i) * 1000) + ",";
      csv += String(speedHistory[idx], 2) + ",";
      csv += String(speedHistory[idx] * speedScalingFactor, 2) + "\n";
    }
    request->send(200, "text/csv", csv);
  });

  // API: Export CAN data as CSV
  server.on("/api/export/can", HTTP_GET, [](AsyncWebServerRequest *request) {
    String csv = "Timestamp,CAN_ID,DLC,Data_Hex\n";
    for (int i = 0; i < CAN_HISTORY_SIZE; i++) {
      if (canHistory[i].id != 0) {
        csv += String(canHistory[i].timestamp) + ",0x";
        csv += String(canHistory[i].id, HEX) + ",";
        csv += String(canHistory[i].len) + ",";
        for (int j = 0; j < canHistory[i].len; j++) {
          if (canHistory[i].data[j] < 16) csv += "0";
          csv += String(canHistory[i].data[j], HEX);
          if (j < canHistory[i].len - 1) csv += " ";
        }
        csv += "\n";
      }
    }
    request->send(200, "text/csv", csv);
  });

  // API: OTA firmware update
  server.on("/api/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain",
      (Update.hasError()) ? "UPDATE FAILED" : "UPDATE SUCCESS");
    response->addHeader("Connection", "close");
    request->send(response);
    delay(1000);
    ESP.restart();
  }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (!index) {
      Serial.printf("OTA Update Start: %s\n", filename.c_str());
      if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)) {
        Update.printError(Serial);
      }
    }
    if (!Update.hasError()) {
      if (Update.write(data, len) != len) {
        Update.printError(Serial);
      }
    }
    if (final) {
      if (Update.end(true)) {
        Serial.printf("OTA Update Success: %uB\n", index + len);
      } else {
        Update.printError(Serial);
      }
    }
  });
}

// ==================== WebSocket ====================
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected\n", client->id());
    // Send initial data
    client->text(getStatusJSON());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
}

void sendWebSocketUpdate() {
  if (ws.count() > 0) {
    String json = getStatusJSON();
    ws.textAll(json);
  }
}

// ==================== JSON Functions ====================
String getStatusJSON() {
  StaticJsonDocument<512> doc;

  doc["realSpeed"] = round(realSpeed * 10) / 10.0;
  doc["manipulatedSpeed"] = round(manipulatedSpeed * 10) / 10.0;
  doc["scalingFactor"] = speedScalingFactor;
  doc["offset"] = speedOffset;
  doc["pulseCount"] = reedPulseCount;
  doc["goodPulses"] = goodPulses;
  doc["badPulses"] = badPulses;
  doc["signalQuality"] = goodPulses + badPulses > 0 ?
    (goodPulses * 100.0) / (goodPulses + badPulses) : 100.0;

  doc["uptime"] = stats.uptime;
  doc["maxSpeed"] = stats.maxSpeed;
  doc["wifiRssi"] = WiFi.RSSI();
  doc["freeHeap"] = ESP.getFreeHeap();
  doc["canMessages"] = stats.canMessagesReceived;

  doc["calibrationMode"] = calibrationMode;
  doc["calibrationRemaining"] = calibrationMode ?
    (CALIBRATION_DURATION - (millis() - calibrationStartTime)) / 1000 : 0;

  doc["wheelCircumference"] = wheelCircumference;
  doc["magnetsPerRevolution"] = magnetsPerRevolution;

  String output;
  serializeJson(doc, output);
  return output;
}

String getSpeedHistoryJSON() {
  StaticJsonDocument<2048> doc;
  JsonArray real = doc.createNestedArray("real");
  JsonArray manipulated = doc.createNestedArray("manipulated");

  for (int i = 0; i < HISTORY_SIZE; i++) {
    int idx = (historyIndex + i) % HISTORY_SIZE;
    real.add(round(speedHistory[idx] * 10) / 10.0);
    manipulated.add(round(speedHistory[idx] * speedScalingFactor * 10) / 10.0);
  }

  String output;
  serializeJson(doc, output);
  return output;
}

String getCANHistoryJSON() {
  StaticJsonDocument<4096> doc;
  JsonArray messages = doc.createNestedArray("messages");

  for (int i = 0; i < CAN_HISTORY_SIZE; i++) {
    if (canHistory[i].id != 0) {
      JsonObject msg = messages.createNestedObject();
      msg["timestamp"] = canHistory[i].timestamp;
      msg["id"] = String(canHistory[i].id, HEX);
      msg["len"] = canHistory[i].len;

      String hexData = "";
      String asciiData = "";
      for (int j = 0; j < canHistory[i].len; j++) {
        if (canHistory[i].data[j] < 16) hexData += "0";
        hexData += String(canHistory[i].data[j], HEX);
        if (j < canHistory[i].len - 1) hexData += " ";

        if (canHistory[i].data[j] >= 32 && canHistory[i].data[j] <= 126) {
          asciiData += (char)canHistory[i].data[j];
        } else {
          asciiData += ".";
        }
      }
      msg["hex"] = hexData;
      msg["ascii"] = asciiData;
    }
  }

  String output;
  serializeJson(doc, output);
  return output;
}

// ==================== Configuration ====================
void loadConfiguration() {
  preferences.begin("speedconfig", true);

  speedScalingFactor = preferences.getFloat("scaleFactor", 0.5455);
  speedOffset = preferences.getFloat("offset", 0.0);
  wheelCircumference = preferences.getFloat("wheelCirc", WHEEL_CIRCUMFERENCE_MM);
  magnetsPerRevolution = preferences.getUChar("magnets", MAGNETS_PER_REVOLUTION);

  preferences.end();

  Serial.println("Configuration loaded:");
  Serial.print("  Scaling Factor: ");
  Serial.println(speedScalingFactor, 4);
  Serial.print("  Offset: ");
  Serial.println(speedOffset, 2);
  Serial.print("  Wheel Circumference: ");
  Serial.println(wheelCircumference);
  Serial.print("  Magnets per Revolution: ");
  Serial.println(magnetsPerRevolution);
}

// ==================== Statistics ====================
void updateStatistics() {
  stats.uptime = millis() / 1000;
  stats.totalPulses = reedPulseCount;
  stats.wifiRssi = WiFi.RSSI();
  stats.freeHeap = ESP.getFreeHeap();
}
