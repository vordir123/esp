/*
 * Teensy 4.1 CAN Logger & ESP32 Bridge
 *
 * Hardware:
 * - Teensy 4.1
 * - CAN Transceiver (MCP2562) on CAN1 (TX: Pin 22, RX: Pin 23)
 * - 16GB SD Card (Class 10, FAT32)
 * - I2C Connection to ESP32 (SDA: Pin 18, SCL: Pin 19)
 * - Status LEDs: CAN Activity (Pin 2), SD Write (Pin 3), Error (Pin 4)
 *
 * Features:
 * - CAN Bus logging at 500kbps
 * - Rotating log files on SD card
 * - I2C communication with ESP32
 * - Real-time CAN message filtering
 * - Robust error handling and recovery
 */

#include <FlexCAN_T4.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// ==================== Configuration ====================
#define CAN_BAUDRATE 500000
#define I2C_ADDRESS 0x42
#define LOG_FILE_MAX_SIZE (10 * 1024 * 1024)  // 10 MB per file
#define MAX_LOG_FILES 100
#define SD_CS_PIN BUILTIN_SDCARD
#define BUFFER_SIZE 256

// LED Pins
#define LED_CAN_ACTIVITY 2
#define LED_SD_WRITE 3
#define LED_ERROR 4

// ==================== Global Objects ====================
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
File logFile;

// ==================== Structures ====================
struct CANMessage {
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
  uint32_t timestamp;
};

// ==================== Circular Buffer ====================
volatile CANMessage canBuffer[BUFFER_SIZE];
volatile uint16_t bufferHead = 0;
volatile uint16_t bufferTail = 0;
volatile bool bufferOverflow = false;

// ==================== I2C Message Buffer ====================
struct I2CMessage {
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
} i2cBuffer;
volatile bool i2cDataReady = false;

// ==================== Statistics ====================
struct Stats {
  uint32_t messagesReceived;
  uint32_t messagesLogged;
  uint32_t messagesSentToESP32;
  uint32_t sdErrors;
  uint32_t bufferOverflows;
  uint32_t uptime;
} stats = {0};

// ==================== State Variables ====================
char currentLogFileName[32];
uint32_t currentLogFileSize = 0;
uint8_t currentLogFileNumber = 1;
bool sdCardAvailable = false;
uint32_t lastActivityBlink = 0;
uint32_t lastSDWriteBlink = 0;
uint32_t lastStatsOutput = 0;
uint32_t lastHeartbeat = 0;

// ==================== CAN Message Filter ====================
// Add important CAN IDs to forward to ESP32
const uint32_t PRIORITY_CAN_IDS[] = {
  0x100, 0x200, 0x201, 0x300, // Example: Speed, RPM, etc.
  0x400, 0x500
};
const uint8_t NUM_PRIORITY_IDS = sizeof(PRIORITY_CAN_IDS) / sizeof(PRIORITY_CAN_IDS[0]);

// ==================== Setup ====================
void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Teensy 4.1 CAN Logger Starting ===");

  // Initialize LEDs
  pinMode(LED_CAN_ACTIVITY, OUTPUT);
  pinMode(LED_SD_WRITE, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);

  // LED startup sequence
  digitalWrite(LED_CAN_ACTIVITY, HIGH);
  delay(200);
  digitalWrite(LED_SD_WRITE, HIGH);
  delay(200);
  digitalWrite(LED_ERROR, HIGH);
  delay(200);
  digitalWrite(LED_CAN_ACTIVITY, LOW);
  digitalWrite(LED_SD_WRITE, LOW);
  digitalWrite(LED_ERROR, LOW);

  // Initialize I2C as slave
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(i2cRequestHandler);
  Serial.println("I2C initialized as slave at 0x42");

  // Initialize SD Card
  sdCardAvailable = initSDCard();
  if (sdCardAvailable) {
    Serial.println("SD Card initialized successfully");
    createNewLogFile();
  } else {
    Serial.println("ERROR: SD Card initialization failed!");
    digitalWrite(LED_ERROR, HIGH);
  }

  // Initialize CAN Bus
  if (initCANBus()) {
    Serial.println("CAN Bus initialized successfully");
  } else {
    Serial.println("ERROR: CAN Bus initialization failed!");
    digitalWrite(LED_ERROR, HIGH);
  }

  Serial.println("=== System Ready ===");
  Serial.println("Logging CAN messages...");
}

// ==================== Main Loop ====================
void loop() {
  uint32_t currentMillis = millis();

  // Process CAN messages from buffer
  processCANBuffer();

  // Periodic statistics output
  if (currentMillis - lastStatsOutput >= 10000) {
    printStatistics();
    lastStatsOutput = currentMillis;
  }

  // Heartbeat LED
  if (currentMillis - lastHeartbeat >= 1000) {
    blinkLED(LED_CAN_ACTIVITY, 50);
    lastHeartbeat = currentMillis;
    stats.uptime = currentMillis / 1000;
  }

  // Turn off activity LEDs after blink duration
  if (currentMillis - lastActivityBlink >= 50) {
    digitalWrite(LED_CAN_ACTIVITY, LOW);
  }
  if (currentMillis - lastSDWriteBlink >= 50) {
    digitalWrite(LED_SD_WRITE, LOW);
  }

  // Check if log file needs rotation
  if (sdCardAvailable && currentLogFileSize >= LOG_FILE_MAX_SIZE) {
    rotateLogFile();
  }

  // Attempt SD card recovery if failed
  if (!sdCardAvailable && currentMillis % 30000 == 0) {
    Serial.println("Attempting SD card recovery...");
    sdCardAvailable = initSDCard();
    if (sdCardAvailable) {
      createNewLogFile();
      digitalWrite(LED_ERROR, LOW);
    }
  }
}

// ==================== CAN Bus Functions ====================
bool initCANBus() {
  can1.begin();
  can1.setBaudRate(CAN_BAUDRATE);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canSniff);
  can1.mailboxStatus();

  return true;
}

void canSniff(const CAN_message_t &msg) {
  // Add to circular buffer
  uint16_t nextHead = (bufferHead + 1) % BUFFER_SIZE;

  if (nextHead == bufferTail) {
    // Buffer full - overflow
    bufferOverflow = true;
    stats.bufferOverflows++;
    return;
  }

  // Store message in buffer
  canBuffer[bufferHead].id = msg.id;
  canBuffer[bufferHead].len = msg.len;
  memcpy((void*)canBuffer[bufferHead].data, msg.buf, msg.len);
  canBuffer[bufferHead].timestamp = millis();

  bufferHead = nextHead;
  stats.messagesReceived++;

  // Blink CAN activity LED
  digitalWrite(LED_CAN_ACTIVITY, HIGH);
  lastActivityBlink = millis();
}

void processCANBuffer() {
  while (bufferTail != bufferHead) {
    // Get message from buffer
    CANMessage msg = canBuffer[bufferTail];
    bufferTail = (bufferTail + 1) % BUFFER_SIZE;

    // Log to SD card
    if (sdCardAvailable) {
      logMessageToSD(msg);
    }

    // Check if this is a priority message for ESP32
    if (isPriorityMessage(msg.id)) {
      sendMessageToESP32(msg);
    }
  }
}

bool isPriorityMessage(uint32_t canId) {
  for (uint8_t i = 0; i < NUM_PRIORITY_IDS; i++) {
    if (canId == PRIORITY_CAN_IDS[i]) {
      return true;
    }
  }
  return false;
}

void sendMessageToESP32(const CANMessage &msg) {
  // Prepare I2C buffer
  noInterrupts();
  i2cBuffer.id = msg.id;
  i2cBuffer.len = msg.len;
  memcpy((void*)i2cBuffer.data, (void*)msg.data, msg.len);
  i2cDataReady = true;
  interrupts();

  stats.messagesSentToESP32++;
}

// ==================== I2C Functions ====================
void i2cRequestHandler() {
  if (i2cDataReady) {
    // Send CAN message over I2C
    Wire.write((uint8_t*)&i2cBuffer, sizeof(i2cBuffer));
    i2cDataReady = false;
  } else {
    // Send empty/idle message
    uint8_t emptyMsg[sizeof(i2cBuffer)] = {0};
    Wire.write(emptyMsg, sizeof(emptyMsg));
  }
}

// ==================== SD Card Functions ====================
bool initSDCard() {
  Serial.println("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    return false;
  }

  uint64_t cardSize = SD.totalSize();
  uint64_t cardUsed = SD.usedSize();

  Serial.print("SD Card Size: ");
  Serial.print(cardSize / (1024 * 1024));
  Serial.println(" MB");
  Serial.print("SD Card Used: ");
  Serial.print(cardUsed / (1024 * 1024));
  Serial.println(" MB");

  return true;
}

void createNewLogFile() {
  // Find next available log file number
  while (currentLogFileNumber <= MAX_LOG_FILES) {
    sprintf(currentLogFileName, "canlog_%03d.csv", currentLogFileNumber);

    if (!SD.exists(currentLogFileName)) {
      break;
    }
    currentLogFileNumber++;
  }

  if (currentLogFileNumber > MAX_LOG_FILES) {
    // Wrap around to start
    currentLogFileNumber = 1;
    sprintf(currentLogFileName, "canlog_%03d.csv", currentLogFileNumber);
    SD.remove(currentLogFileName);
  }

  // Create new log file
  logFile = SD.open(currentLogFileName, FILE_WRITE);

  if (logFile) {
    // Write CSV header
    logFile.println("Timestamp,CAN_ID,DLC,Data_Hex,Data_ASCII");
    logFile.flush();
    currentLogFileSize = logFile.size();

    Serial.print("Created log file: ");
    Serial.println(currentLogFileName);
  } else {
    Serial.println("ERROR: Could not create log file!");
    sdCardAvailable = false;
    digitalWrite(LED_ERROR, HIGH);
    stats.sdErrors++;
  }
}

void rotateLogFile() {
  Serial.println("Rotating log file...");

  if (logFile) {
    logFile.close();
  }

  currentLogFileNumber++;
  currentLogFileSize = 0;
  createNewLogFile();
}

void logMessageToSD(const CANMessage &msg) {
  if (!logFile) {
    return;
  }

  // Format: Timestamp, CAN_ID, DLC, Data_Hex, Data_ASCII
  char logLine[128];
  char hexData[32] = "";
  char asciiData[16] = "";

  // Build hex string
  for (uint8_t i = 0; i < msg.len; i++) {
    char hex[4];
    sprintf(hex, "%02X ", msg.data[i]);
    strcat(hexData, hex);

    // Build ASCII string (printable characters only)
    if (msg.data[i] >= 32 && msg.data[i] <= 126) {
      asciiData[i] = msg.data[i];
    } else {
      asciiData[i] = '.';
    }
  }
  asciiData[msg.len] = '\0';

  // Build log line
  sprintf(logLine, "%lu,0x%03X,%d,%s,%s",
          msg.timestamp, msg.id, msg.len, hexData, asciiData);

  // Write to SD card
  logFile.println(logLine);

  // Periodic flush (every 50 messages)
  if (stats.messagesLogged % 50 == 0) {
    logFile.flush();
    digitalWrite(LED_SD_WRITE, HIGH);
    lastSDWriteBlink = millis();
  }

  currentLogFileSize = logFile.size();
  stats.messagesLogged++;
}

// ==================== Utility Functions ====================
void blinkLED(uint8_t pin, uint16_t duration) {
  digitalWrite(pin, HIGH);
}

void printStatistics() {
  Serial.println("\n=== Statistics ===");
  Serial.print("Uptime: ");
  Serial.print(stats.uptime);
  Serial.println(" seconds");
  Serial.print("CAN Messages Received: ");
  Serial.println(stats.messagesReceived);
  Serial.print("Messages Logged: ");
  Serial.println(stats.messagesLogged);
  Serial.print("Messages to ESP32: ");
  Serial.println(stats.messagesSentToESP32);
  Serial.print("Buffer Overflows: ");
  Serial.println(stats.bufferOverflows);
  Serial.print("SD Errors: ");
  Serial.println(stats.sdErrors);
  Serial.print("Current Log File: ");
  Serial.println(currentLogFileName);
  Serial.print("Log File Size: ");
  Serial.print(currentLogFileSize / 1024);
  Serial.println(" KB");

  if (bufferOverflow) {
    Serial.println("WARNING: Buffer overflow detected!");
    bufferOverflow = false;
  }

  Serial.println("==================\n");
}
