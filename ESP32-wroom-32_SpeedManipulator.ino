/*
 * ESP32-WROOM-32 Speed Manipulator with CAN Dashboard
 * Features:
 * - Reed sensor speed detection with manipulation
 * - Web dashboard with real-time monitoring
 * - CAN data display from Teensy 4.1
 * - Calibration mode and data export
 * - OTA firmware updates
 */

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Pin Definitions
const int REED_PIN = 18;          // Reed sensor input with pull-up
const int PWM_PIN = 19;           // PWM output for manipulated speed
const int STATUS_LED = 2;         // Built-in LED for status
const int I2C_SDA = 21;           // I2C data line to Teensy
const int I2C_SCL = 22;           // I2C clock line to Teensy

// PWM Configuration
const int PWM_CHANNEL = 0;
const int PWM_FREQUENCY = 1000;  // 1kHz PWM frequency
const int PWM_RESOLUTION = 12;   // 12-bit resolution (0-4095)

// WiFi Credentials
const char* WIFI_SSID = "YourWiFiSSID";
const char* WIFI_PASSWORD = "YourWiFiPassword";
const char* HOSTNAME = "ESP32-SpeedManipulator";

// Web Server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Preferences for NVS storage
Preferences preferences;

// Speed Calculation Variables
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile unsigned long pulseCount = 0;
float wheelCircumference = 2.17;  // meters (27.5 inch wheel)
int magnetsPerRevolution = 1;
float realSpeed = 0.0;
float manipulatedSpeed = 0.0;
float speedScaleFactor = 0.545;   // Default: 12/22 = 0.545
float speedOffset = -10.0;        // Linear offset for manipulation

// Calibration Mode
bool calibrationMode = false;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_DURATION = 30000; // 30 seconds
const float CALIBRATION_SPEED = 15.0; // km/h

// System Statistics
unsigned long systemUptime = 0;
float cpuTemperature = 0.0;
int freeHeapSize = 0;
int wifiRSSI = 0;

// Speed History Buffer (60 seconds at 1Hz)
const int HISTORY_SIZE = 60;
float speedHistory[HISTORY_SIZE];
float manipulatedHistory[HISTORY_SIZE];
int historyIndex = 0;

// CAN Data from Teensy
struct CANMessage {
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
    unsigned long timestamp;
};

QueueHandle_t canMessageQueue;
const int CAN_QUEUE_SIZE = 50;

// I2C Communication Protocol
const uint8_t TEENSY_I2C_ADDRESS = 0x08;
const uint8_t CMD_CAN_DATA = 0x01;
const uint8_t CMD_STATUS = 0x02;
const uint8_t CMD_CONFIG = 0x03;

// Authentication
const char* WEB_USERNAME = "admin";
const char* WEB_PASSWORD = "esp32speed";

// Function Prototypes
void IRAM_ATTR onReedPulse();
void setupWiFi();
void setupWebServer();
void setupOTA();
void speedCalculationTask(void *parameter);
void pwmOutputTask(void *parameter);
void systemMonitorTask(void *parameter);
void i2cCommunicationTask(void *parameter);
void webSocketTask(void *parameter);
float calculateManipulatedSpeed(float realSpeed);
String getSystemStats();
String getSpeedHistory();
void handleCalibration();
void savePreferences();
void loadPreferences();

// Reed Sensor Interrupt Handler
void IRAM_ATTR onReedPulse() {
    unsigned long currentTime = micros();
    unsigned long interval = currentTime - lastPulseTime;
    
    // Debounce: ignore pulses faster than 10ms
    if (interval > 10000) {
        pulseInterval = interval;
        lastPulseTime = currentTime;
        pulseCount++;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Speed Manipulator Starting...");
    
    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
    }
    
    // Load saved preferences
    loadPreferences();
    
    // Configure pins
    pinMode(REED_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(REED_PIN), onReedPulse, FALLING);
    
    // Configure PWM
    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PWM_PIN, PWM_CHANNEL);
    
    // Initialize I2C as master
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000); // 100kHz I2C clock
    
    // Create message queue for CAN data
    canMessageQueue = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CANMessage));
    
    // Setup WiFi and web server
    setupWiFi();
    setupWebServer();
    setupOTA();
    
    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(speedCalculationTask, "SpeedCalc", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(pwmOutputTask, "PWMOutput", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(systemMonitorTask, "SysMonitor", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(i2cCommunicationTask, "I2CComm", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(webSocketTask, "WebSocket", 4096, NULL, 1, NULL, 1);
    
    Serial.println("Setup complete!");
}

void loop() {
    // Handle OTA updates
    ArduinoOTA.handle();
    
    // Handle calibration mode timeout
    if (calibrationMode && (millis() - calibrationStartTime > CALIBRATION_DURATION)) {
        calibrationMode = false;
        Serial.println("Calibration mode ended");
    }
    
    delay(10);
}

// Speed calculation task
void speedCalculationTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (true) {
        // Calculate speed from pulse interval
        if (pulseInterval > 0 && (micros() - lastPulseTime < 2000000)) { // 2 second timeout
            // Speed = (wheelCircumference / magnetsPerRevolution) * (1000000 / pulseInterval) * 3.6
            // Convert from m/s to km/h
            float frequency = 1000000.0 / pulseInterval; // Hz
            realSpeed = (wheelCircumference / magnetsPerRevolution) * frequency * 3.6;
            
            // Apply speed manipulation
            if (!calibrationMode) {
                manipulatedSpeed = calculateManipulatedSpeed(realSpeed);
            } else {
                manipulatedSpeed = CALIBRATION_SPEED;
            }
        } else {
            // No pulses detected - speed is 0
            realSpeed = 0.0;
            manipulatedSpeed = calibrationMode ? CALIBRATION_SPEED : 0.0;
        }
        
        // Update speed history
        speedHistory[historyIndex] = realSpeed;
        manipulatedHistory[historyIndex] = manipulatedSpeed;
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100)); // 10Hz update rate
    }
}

// PWM output task
void pwmOutputTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (true) {
        // Convert manipulated speed to PWM frequency
        // Assuming 1 pulse per km/h (adjustable via calibration)
        float targetFrequency = manipulatedSpeed * preferences.getFloat("pulsesPerKmh", 1.0);
        
        if (targetFrequency > 0) {
            // Calculate PWM duty cycle for square wave (50%)
            int dutyCycle = 2048; // 50% of 4095 (12-bit)
            
            // Update PWM frequency
            ledcSetup(PWM_CHANNEL, targetFrequency, PWM_RESOLUTION);
            ledcWrite(PWM_CHANNEL, dutyCycle);
        } else {
            // Stop PWM output
            ledcWrite(PWM_CHANNEL, 0);
        }
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50)); // 20Hz update rate
    }
}

// System monitoring task
void systemMonitorTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (true) {
        // Update system statistics
        systemUptime = millis() / 1000; // seconds
        cpuTemperature = temperatureRead();
        freeHeapSize = ESP.getFreeHeap();
        wifiRSSI = WiFi.RSSI();
        
        // Blink status LED
        digitalWrite(STATUS_LED, (millis() / 500) % 2);
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000)); // 1Hz update rate
    }
}

// I2C communication task for Teensy data
void i2cCommunicationTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    uint8_t buffer[32];
    
    while (true) {
        // Request CAN data from Teensy
        Wire.beginTransmission(TEENSY_I2C_ADDRESS);
        Wire.write(CMD_CAN_DATA);
        Wire.endTransmission();
        
        // Read response
        Wire.requestFrom(TEENSY_I2C_ADDRESS, 16);
        if (Wire.available() >= 13) { // Minimum CAN message size
            CANMessage msg;
            msg.timestamp = millis();
            
            // Read CAN ID (4 bytes)
            msg.id = Wire.read() | (Wire.read() << 8) | (Wire.read() << 16) | (Wire.read() << 24);
            
            // Read data length
            msg.length = Wire.read();
            if (msg.length > 8) msg.length = 8;
            
            // Read CAN data
            for (int i = 0; i < msg.length; i++) {
                msg.data[i] = Wire.read();
            }
            
            // Add to queue if valid
            if (msg.id != 0xFFFFFFFF) {
                xQueueSend(canMessageQueue, &msg, 0);
            }
        }
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20)); // 50Hz polling rate
    }
}

// WebSocket task for real-time updates
void webSocketTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (true) {
        if (ws.count() > 0) {
            // Prepare JSON data
            StaticJsonDocument<512> doc;
            doc["realSpeed"] = realSpeed;
            doc["manipulatedSpeed"] = manipulatedSpeed;
            doc["pulseCount"] = pulseCount;
            doc["uptime"] = systemUptime;
            doc["freeHeap"] = freeHeapSize;
            doc["wifiRSSI"] = wifiRSSI;
            doc["cpuTemp"] = cpuTemperature;
            doc["calibrationMode"] = calibrationMode;
            
            // Add latest CAN messages
            JsonArray canArray = doc.createNestedArray("canMessages");
            CANMessage msg;
            int msgCount = 0;
            while (xQueueReceive(canMessageQueue, &msg, 0) == pdTRUE && msgCount < 5) {
                JsonObject canObj = canArray.createNestedObject();
                canObj["id"] = msg.id;
                canObj["length"] = msg.length;
                JsonArray dataArray = canObj.createNestedArray("data");
                for (int i = 0; i < msg.length; i++) {
                    dataArray.add(msg.data[i]);
                }
                msgCount++;
            }
            
            String output;
            serializeJson(doc, output);
            ws.textAll(output);
        }
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100)); // 10Hz update rate
    }
}

// Calculate manipulated speed based on real speed
float calculateManipulatedSpeed(float realSpeed) {
    // Linear scaling: 22 km/h real -> 12 km/h output
    // Formula: manipulated = (real * scaleFactor) + offset
    return (realSpeed * speedScaleFactor) + speedOffset;
}

// Setup WiFi connection
void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
}

// Setup web server and routes
void setupWebServer() {
    // WebSocket event handler
    ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, 
                  AwsEventType type, void *arg, uint8_t *data, size_t len) {
        if (type == WS_EVT_CONNECT) {
            Serial.printf("WebSocket client #%u connected\n", client->id());
        } else if (type == WS_EVT_DISCONNECT) {
            Serial.printf("WebSocket client #%u disconnected\n", client->id());
        }
    });
    server.addHandler(&ws);
    
    // Serve dashboard
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->authenticate(WEB_USERNAME, WEB_PASSWORD)) {
            return request->requestAuthentication();
        }
        request->send(SPIFFS, "/dashboard.html", "text/html");
    });
    
    // API endpoints
    server.on("/api/stats", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->authenticate(WEB_USERNAME, WEB_PASSWORD)) {
            return request->requestAuthentication();
        }
        request->send(200, "application/json", getSystemStats());
    });
    
    server.on("/api/history", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->authenticate(WEB_USERNAME, WEB_PASSWORD)) {
            return request->requestAuthentication();
        }
        request->send(200, "application/json", getSpeedHistory());
    });
    
    server.on("/api/calibrate", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!request->authenticate(WEB_USERNAME, WEB_PASSWORD)) {
            return request->requestAuthentication();
        }
        handleCalibration();
        request->send(200, "text/plain", "Calibration started");
    });
    
    server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        if (!request->authenticate(WEB_USERNAME, WEB_PASSWORD)) {
            return request->requestAuthentication();
        }
        
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, data);
        
        if (!error) {
            if (doc.containsKey("wheelCircumference")) {
                wheelCircumference = doc["wheelCircumference"];
                preferences.putFloat("wheelCirc", wheelCircumference);
            }
            if (doc.containsKey("magnetsPerRevolution")) {
                magnetsPerRevolution = doc["magnetsPerRevolution"];
                preferences.putInt("magnets", magnetsPerRevolution);
            }
            if (doc.containsKey("speedScaleFactor")) {
                speedScaleFactor = doc["speedScaleFactor"];
                preferences.putFloat("scaleFactor", speedScaleFactor);
            }
            if (doc.containsKey("speedOffset")) {
                speedOffset = doc["speedOffset"];
                preferences.putFloat("offset", speedOffset);
            }
            savePreferences();
            request->send(200, "text/plain", "Configuration updated");
        } else {
            request->send(400, "text/plain", "Invalid JSON");
        }
    });
    
    server.on("/api/export", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->authenticate(WEB_USERNAME, WEB_PASSWORD)) {
            return request->requestAuthentication();
        }
        
        String csv = "Timestamp,Real Speed (km/h),Manipulated Speed (km/h),Pulse Count\n";
        for (int i = 0; i < HISTORY_SIZE; i++) {
            csv += String(i) + "," + String(speedHistory[i]) + "," + 
                   String(manipulatedHistory[i]) + "," + String(pulseCount) + "\n";
        }
        
        request->send(200, "text/csv", csv);
    });
    
    // Start server
    server.begin();
    Serial.println("Web server started");
}

// Setup OTA updates
void setupOTA() {
    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.setPassword("esp32ota");
    
    ArduinoOTA.onStart([]() {
        Serial.println("OTA update started");
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA update completed");
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    
    ArduinoOTA.begin();
    Serial.println("OTA ready");
}

// Get system statistics as JSON
String getSystemStats() {
    StaticJsonDocument<512> doc;
    doc["uptime"] = systemUptime;
    doc["freeHeap"] = freeHeapSize;
    doc["cpuTemp"] = cpuTemperature;
    doc["wifiRSSI"] = wifiRSSI;
    doc["pulseCount"] = pulseCount;
    doc["realSpeed"] = realSpeed;
    doc["manipulatedSpeed"] = manipulatedSpeed;
    doc["wheelCircumference"] = wheelCircumference;
    doc["magnetsPerRevolution"] = magnetsPerRevolution;
    doc["speedScaleFactor"] = speedScaleFactor;
    doc["speedOffset"] = speedOffset;
    
    String output;
    serializeJson(doc, output);
    return output;
}

// Get speed history as JSON
String getSpeedHistory() {
    StaticJsonDocument<2048> doc;
    JsonArray realArray = doc.createNestedArray("realSpeed");
    JsonArray manipArray = doc.createNestedArray("manipulatedSpeed");
    
    for (int i = 0; i < HISTORY_SIZE; i++) {
        int idx = (historyIndex + i) % HISTORY_SIZE;
        realArray.add(speedHistory[idx]);
        manipArray.add(manipulatedHistory[idx]);
    }
    
    String output;
    serializeJson(doc, output);
    return output;
}

// Handle calibration mode
void handleCalibration() {
    calibrationMode = true;
    calibrationStartTime = millis();
    Serial.println("Calibration mode started - outputting 15 km/h for 30 seconds");
}

// Save preferences to NVS
void savePreferences() {
    preferences.begin("speed-manip", false);
    preferences.putFloat("wheelCirc", wheelCircumference);
    preferences.putInt("magnets", magnetsPerRevolution);
    preferences.putFloat("scaleFactor", speedScaleFactor);
    preferences.putFloat("offset", speedOffset);
    preferences.end();
}

// Load preferences from NVS
void loadPreferences() {
    preferences.begin("speed-manip", false);
    wheelCircumference = preferences.getFloat("wheelCirc", 2.17);
    magnetsPerRevolution = preferences.getInt("magnets", 1);
    speedScaleFactor = preferences.getFloat("scaleFactor", 0.545);
    speedOffset = preferences.getFloat("offset", -10.0);
    preferences.end();
}