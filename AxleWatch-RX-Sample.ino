/*
 * AxleWatch Receiver Firmware v1.0
 * ESP32-S3 Production Firmware
 *
 * Features:
 * - LoRa 433MHz reception from multiple transmitters
 * - ILI9341 touchscreen display with live temperature monitoring
 * - GPS integration with TinyGPS++
 * - SD card logging (100+ hours)
 * - WiFi AP mode (60s on boot) + web configuration portal
 * - Cloud upload to AxleWatch.com API
 * - Configurable alarm thresholds with buzzer alerts
 * - Multi-transmitter auto-cycling display
 *
 * Hardware: ESP32-S3, ILI9341, XPT2046, SX1276 LoRa, GPS module
 */

// ======================== INCLUDES ========================
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <FS.h>
#include <time.h>

// ======================== PIN DEFINITIONS ========================
// ESP32-S3 Pin Assignments - AxleWatch Receiver Hardware

// ============= DISPLAY PINS (Default SPI Bus) =============
#define TFT_CS          8
#define TFT_DC          7
#define TFT_RST         9

// ============= TOUCH PINS (Shared Default SPI) =============
#define TOUCH_CS        5
#define TOUCH_IRQ       2

// Touch calibration values (hardware-specific)
#define TOUCH_MIN_X     520
#define TOUCH_MAX_X     3748
#define TOUCH_MIN_Y     449
#define TOUCH_MAX_Y     3538

// ============= LORA PINS (Dedicated SPI Bus) =============
#define LORA_CS         10
#define LORA_MOSI       11
#define LORA_MISO       12
#define LORA_SCK        13
#define LORA_DIO0       14
#define LORA_RST        4

// ============= SD CARD PINS (Dedicated SPI Bus) =============
#define SD_CS_PIN       39
#define SD_MISO_PIN     37
#define SD_SCK_PIN      36
#define SD_MOSI_PIN     35

// ============= GPS PINS (Hardware Serial2) =============
#define GPS_RX_PIN      18
#define GPS_TX_PIN      17

// ============= LED PINS =============
#define LED1            21  // System status LED
#define LED2            26  // GPS/Activity status LED
// Note: GPIO 6 is used by display, not a status LED

// ============= BUZZER PIN =============
#define BUZZER_PIN      15

// ======================== CONSTANTS ========================
#define LORA_FREQUENCY      433E6
#define LORA_SF             7
#define LORA_BW             125E3

#define NUM_TEMP_SENSORS    9
#define MAX_TRANSMITTERS    10

#define DEFAULT_WARN_OFFSET     40.0
#define DEFAULT_CRIT_OFFSET     60.0

#define AP_MODE_DURATION        60000  // 60 seconds
#define CLOUD_UPLOAD_INTERVAL   60000  // 60 seconds
#define DISPLAY_CYCLE_INTERVAL  5000   // 5 seconds
#define GPS_UPDATE_INTERVAL     1000   // 1 second

#define AP_SSID                 "AxleWatch-Setup"
#define AP_PASSWORD             "axlewatch123"

// Set to false to skip SD card initialization (if causing hangs)
#define ENABLE_SD_CARD          true

// ======================== GLOBAL OBJECTS ========================
// SPI Bus Instances (ESP32-S3 has FSPI and HSPI)
// Per schematic: Display + Touch + SD share FSPI, LoRa uses HSPI
SPIClass hspi(HSPI);  // Dedicated SPI bus for LoRa only
// Default SPI (FSPI) used for Display + Touch + SD Card

// Peripheral objects
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
WebServer webServer(80);
Preferences prefs;

// ======================== DATA STRUCTURES ========================

struct TransmitterData {
  char txID[16];
  float temps[NUM_TEMP_SENSORS];
  float ambientTemp;
  unsigned long lastReceived;
  int rssi;
  bool active;
  uint8_t alarmLevels[NUM_TEMP_SENSORS]; // 0=OK, 1=WARN, 2=CRIT
};

struct GPSData {
  double latitude;
  double longitude;
  float speedKmh;
  float course;
  int satellites;
  bool validFix;
  unsigned long timestamp;
};

struct SystemConfig {
  char wifiSSID[64];
  char wifiPassword[64];
  char cloudEndpoint[128];
  char deviceID[32];
  char apiKey[128];  // API key for cloud authentication
  float warnOffset;
  float critOffset;
  bool cloudEnabled;
};

// ======================== CONFIGURATION MANAGER ========================
class ConfigManager {
public:
  SystemConfig config;

  void begin() {
    prefs.begin("axlewatch", false);
    loadConfig();

    // Generate device ID from MAC if not set OR if it's the invalid all-zeros ID
    if (strlen(config.deviceID) == 0 || strcmp(config.deviceID, "AW-000000000000") == 0) {
      // Initialize WiFi to get MAC address
      WiFi.mode(WIFI_STA);
      delay(100); // Give WiFi time to initialize

      uint8_t mac[6];
      WiFi.macAddress(mac);
      snprintf(config.deviceID, sizeof(config.deviceID),
               "AW-%02X%02X%02X%02X%02X%02X",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      saveConfig();

      Serial.print("Generated device ID from MAC: ");
      Serial.println(config.deviceID);
    }
  }

  void loadConfig() {
    prefs.getString("wifiSSID", config.wifiSSID, sizeof(config.wifiSSID));
    prefs.getString("wifiPass", config.wifiPassword, sizeof(config.wifiPassword));
    prefs.getString("cloudURL", config.cloudEndpoint, sizeof(config.cloudEndpoint));
    prefs.getString("deviceID", config.deviceID, sizeof(config.deviceID));
    prefs.getString("apiKey", config.apiKey, sizeof(config.apiKey));

    config.warnOffset = prefs.getFloat("warnOffset", DEFAULT_WARN_OFFSET);
    config.critOffset = prefs.getFloat("critOffset", DEFAULT_CRIT_OFFSET);
    config.cloudEnabled = prefs.getBool("cloudEn", true);

    // Default cloud endpoint if not set
    if (strlen(config.cloudEndpoint) == 0) {
      strcpy(config.cloudEndpoint, "https://axlewatch.com/api/telemetry");
    }
  }

  void saveConfig() {
    prefs.putString("wifiSSID", config.wifiSSID);
    prefs.putString("wifiPass", config.wifiPassword);
    prefs.putString("cloudURL", config.cloudEndpoint);
    prefs.putString("deviceID", config.deviceID);
    prefs.putString("apiKey", config.apiKey);
    prefs.putFloat("warnOffset", config.warnOffset);
    prefs.putFloat("critOffset", config.critOffset);
    prefs.putBool("cloudEn", config.cloudEnabled);
  }

  const char* getDeviceID() { return config.deviceID; }
};

// ======================== LORA MANAGER ========================
class LoRaManager {
public:
  void begin() {
    Serial.println("Initializing LoRa module...");
    Serial.print("  Dedicated HSPI Bus: SCK=");
    Serial.print(LORA_SCK);
    Serial.print(", MISO=");
    Serial.print(LORA_MISO);
    Serial.print(", MOSI=");
    Serial.println(LORA_MOSI);
    Serial.print("  CS=");
    Serial.print(LORA_CS);
    Serial.print(" RST=");
    Serial.print(LORA_RST);
    Serial.print(" DIO0=");
    Serial.println(LORA_DIO0);

    // Initialize dedicated HSPI bus for LoRa
    hspi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

    // Configure LoRa library to use HSPI
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
    LoRa.setSPI(hspi);

    Serial.print("  Frequency: ");
    Serial.print(LORA_FREQUENCY / 1E6);
    Serial.println(" MHz");

    if (!LoRa.begin(LORA_FREQUENCY)) {
      Serial.println("  ERROR: LoRa.begin() failed!");
      Serial.println("  Check: HSPI wiring, CS pin, module power, frequency");
      return;
    }

    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.disableCrc(); // Match transmitter configuration (no CRC)

    Serial.print("  SUCCESS: LoRa initialized - SF");
    Serial.print(LORA_SF);
    Serial.print(", BW");
    Serial.print(LORA_BW / 1000);
    Serial.println(" kHz");
  }

  bool receivePacket(char* buffer, int maxLen, int* rssi) {
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0) return false;

    int idx = 0;
    while (LoRa.available() && idx < maxLen - 1) {
      buffer[idx++] = (char)LoRa.read();
    }
    buffer[idx] = '\0';

    *rssi = LoRa.packetRssi();
    return true;
  }

  bool parsePacket(const char* packet, TransmitterData* tx) {
    // Expected format: TX<ID>:pos1,pos2,...,pos9,ambient OR TRAILER<ID>:... OR DOLLY<ID>:...
    // Example: TX001:45.2,46.1,0.0,0.0,44.8,45.5,0.0,0.0,0.0,22.5
    // Example: TRAILER1:23.5,23.4,23.5,0.0,0.0,0.0,0.0,0.0,0.0,23.3
    // Example: DOLLY2:23.8,22.6,22.7,0.0,0.0,0.0,0.0,0.0,0.0,22.8

    char buffer[128];
    strncpy(buffer, packet, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    // VALIDATION: Check for invalid characters (detect corruption early)
    // Valid chars: A-Z, a-z, 0-9, '.', ',', ':', '-' (for negative temps)
    for (int i = 0; buffer[i] != '\0'; i++) {
      char c = buffer[i];
      bool valid = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                   (c >= '0' && c <= '9') || c == '.' || c == ',' ||
                   c == ':' || c == '-';
      if (!valid) {
        Serial.printf("Invalid character detected at position %d: 0x%02X ('%c')\n", i, c, c);
        return false; // Corrupted packet
      }
    }

    // Accept "TX", "TRAILER", or "DOLLY" prefixes
    bool validPrefix = (strncmp(buffer, "TX", 2) == 0) ||
                       (strncmp(buffer, "TRAILER", 7) == 0) ||
                       (strncmp(buffer, "DOLLY", 5) == 0);
    if (!validPrefix) return false;

    // Extract transmitter ID (everything before the colon)
    char* colonPos = strchr(buffer, ':');
    if (!colonPos) return false;

    int idLen = colonPos - buffer;
    if (idLen >= sizeof(tx->txID)) return false;

    strncpy(tx->txID, buffer, idLen);
    tx->txID[idLen] = '\0';

    // VALIDATION: Count commas - should have exactly 9 for 10 temperature values
    char* dataStart = colonPos + 1;
    int commaCount = 0;
    for (char* p = dataStart; *p; p++) {
      if (*p == ',') commaCount++;
    }
    if (commaCount != 9) {
      Serial.printf("Invalid comma count: expected 9, got %d\n", commaCount);
      return false; // Corrupted packet
    }

    // VALIDATION: Check for double decimals (e.g., "23.3.4")
    char* p = dataStart;
    int dotCount = 0;
    while (*p) {
      if (*p == '.') {
        dotCount++;
        if (dotCount > 1) {
          Serial.println("Double decimal point detected - corrupted value");
          return false;
        }
      } else if (*p == ',') {
        dotCount = 0; // Reset for next value
      }
      p++;
    }

    // Parse temperatures (9 sensors + 1 ambient = 10 values)
    int count = 0;
    char* token = strtok(dataStart, ",");

    while (token && count < NUM_TEMP_SENSORS) {
      tx->temps[count] = atof(token);
      count++;
      token = strtok(NULL, ",");
    }

    // Last value is ambient
    if (token) {
      tx->ambientTemp = atof(token);
    } else {
      return false; // Must have all 10 values
    }

    if (count != NUM_TEMP_SENSORS) return false;

    tx->lastReceived = millis();
    tx->active = true;

    return true;
  }
};

// ======================== GPS MANAGER ========================
class GPSManager {
public:
  GPSData data;

  void begin() {
    Serial.println("Initializing GPS module...");
    Serial.print("  RX=");
    Serial.print(GPS_RX_PIN);
    Serial.print(", TX=");
    Serial.println(GPS_TX_PIN);

    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("  GPS serial initialized at 9600 baud");
  }

  void update() {
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      if (gps.encode(c)) {
        if (gps.location.isValid()) {
          data.latitude = gps.location.lat();
          data.longitude = gps.location.lng();
          data.validFix = true;
        }

        if (gps.speed.isValid()) {
          data.speedKmh = gps.speed.kmph();
        }

        if (gps.course.isValid()) {
          data.course = gps.course.deg();
        }

        if (gps.satellites.isValid()) {
          data.satellites = gps.satellites.value();
        }

        data.timestamp = millis();
      }
    }
  }

  // Get Unix timestamp from GPS date/time, or fallback to millis()
  unsigned long getUnixTimestamp() {
    if (gps.date.isValid() && gps.time.isValid()) {
      // Convert GPS date/time to Unix timestamp
      struct tm timeinfo;
      timeinfo.tm_year = gps.date.year() - 1900;
      timeinfo.tm_mon = gps.date.month() - 1;
      timeinfo.tm_mday = gps.date.day();
      timeinfo.tm_hour = gps.time.hour();
      timeinfo.tm_min = gps.time.minute();
      timeinfo.tm_sec = gps.time.second();

      time_t timestamp = mktime(&timeinfo);
      return (unsigned long)timestamp;
    }
    // Fallback to millis() if GPS time not available
    return millis() / 1000; // Convert to seconds
  }

  GPSData* getData() { return &data; }
};

// ======================== ALARM MANAGER ========================
class AlarmManager {
public:
  bool alarmActive;
  bool alarmMuted;
  bool alarmAcknowledged;
  uint8_t currentAlarmLevel; // 0=none, 1=warn, 2=crit

  unsigned long lastBuzzerToggle;
  bool buzzerState;

  void begin() {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    alarmActive = false;
    alarmMuted = false;
    alarmAcknowledged = false;
    currentAlarmLevel = 0;
    lastBuzzerToggle = 0;
    buzzerState = false;
  }

  void update(TransmitterData transmitters[], int numTx, float warnOffset, float critOffset) {
    // Check all transmitters for alarm conditions
    uint8_t maxLevel = 0;

    for (int i = 0; i < numTx; i++) {
      if (!transmitters[i].active) continue;

      float ambient = transmitters[i].ambientTemp;

      for (int j = 0; j < NUM_TEMP_SENSORS; j++) {
        float temp = transmitters[i].temps[j];
        if (temp < 1.0) { // Ignore 0.0 (unused sensors)
          transmitters[i].alarmLevels[j] = 0;
          continue;
        }

        float delta = temp - ambient;

        if (delta >= critOffset) {
          transmitters[i].alarmLevels[j] = 2;
          if (maxLevel < 2) maxLevel = 2;
        } else if (delta >= warnOffset) {
          transmitters[i].alarmLevels[j] = 1;
          if (maxLevel < 1) maxLevel = 1;
        } else {
          transmitters[i].alarmLevels[j] = 0;
        }
      }
    }

    currentAlarmLevel = maxLevel;
    alarmActive = (maxLevel > 0);

    // Handle buzzer patterns
    if (!alarmMuted && !alarmAcknowledged && alarmActive) {
      unsigned long interval = (currentAlarmLevel == 2) ? 500 : 1000;

      if (millis() - lastBuzzerToggle >= interval) {
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
        lastBuzzerToggle = millis();
      }
    } else {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerState = false;
    }
  }

  void muteAlarm() {
    alarmMuted = true;
    digitalWrite(BUZZER_PIN, LOW);
  }

  void acknowledgeAlarm() {
    alarmAcknowledged = true;
    digitalWrite(BUZZER_PIN, LOW);
  }

  void resetAlarm() {
    if (!alarmActive) {
      alarmMuted = false;
      alarmAcknowledged = false;
    }
  }

  uint16_t getAlarmColor(uint8_t level) {
    switch (level) {
      case 2: return ILI9341_RED;
      case 1: return ILI9341_YELLOW;
      default: return ILI9341_GREEN;
    }
  }
};

// ======================== SD LOGGER ========================
class SDLogger {
public:
  bool sdAvailable;
  File logFile;

  void begin() {
#if ENABLE_SD_CARD
    Serial.println("Initializing SD card...");
    Serial.println("  Default SPI Bus (shared with Display + Touch on pins 35, 36, 37)");
    Serial.print("  CS pin: ");
    Serial.println(SD_CS_PIN);

    // NOTE: Per working code, SD card shares remapped SPI bus with Display and Touch
    Serial.println("  Attempting SD.begin() with 80MHz clock...");
    Serial.println("  (If system hangs here, set ENABLE_SD_CARD to false and recompile)");

    // Try SD card initialization - this may hang if no card present
    sdAvailable = SD.begin(SD_CS_PIN, SPI, 80000000); // 80 MHz clock

    if (sdAvailable) {
      Serial.println("  SUCCESS: SD card initialized");

      uint64_t cardSize = SD.cardSize() / (1024 * 1024);
      Serial.print("  Card size: ");
      Serial.print(cardSize);
      Serial.println(" MB");

      uint8_t cardType = SD.cardType();
      Serial.print("  Card type: ");
      switch(cardType) {
        case CARD_MMC: Serial.println("MMC"); break;
        case CARD_SD: Serial.println("SD"); break;
        case CARD_SDHC: Serial.println("SDHC"); break;
        default: Serial.println("UNKNOWN"); break;
      }
    } else {
      Serial.println("  WARNING: SD card not detected or init failed");
      Serial.println("  Continuing without SD card logging...");
      sdAvailable = false;
    }
#else
    Serial.println("SD card initialization DISABLED (ENABLE_SD_CARD = false)");
    Serial.println("  System will run without SD logging");
    sdAvailable = false;
#endif
  }

  void logData(TransmitterData* tx, GPSData* gpsData, int rssi) {
    if (!sdAvailable) return;

    // Open log file in append mode
    logFile = SD.open("/axlewatch_log.csv", FILE_APPEND);
    if (!logFile) {
      Serial.println("Failed to open log file");
      return;
    }

    // Check if file is new (write header)
    if (logFile.size() == 0) {
      logFile.print("Timestamp,TxID,");
      for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
        logFile.print("Temp");
        logFile.print(i + 1);
        logFile.print(",");
      }
      logFile.print("Ambient,Lat,Lon,Speed,Sats,RSSI,");
      for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
        logFile.print("Alarm");
        logFile.print(i + 1);
        logFile.print(",");
      }
      logFile.println();
    }

    // Write data row
    logFile.print(millis());
    logFile.print(",");
    logFile.print(tx->txID);
    logFile.print(",");

    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
      logFile.print(tx->temps[i], 1);
      logFile.print(",");
    }

    logFile.print(tx->ambientTemp, 1);
    logFile.print(",");
    logFile.print(gpsData->latitude, 6);
    logFile.print(",");
    logFile.print(gpsData->longitude, 6);
    logFile.print(",");
    logFile.print(gpsData->speedKmh, 1);
    logFile.print(",");
    logFile.print(gpsData->satellites);
    logFile.print(",");
    logFile.print(rssi);
    logFile.print(",");

    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
      logFile.print(tx->alarmLevels[i]);
      logFile.print(",");
    }

    logFile.println();
    logFile.close();
  }
};

// ======================== DISPLAY MANAGER ========================
class DisplayManager {
public:
  int currentTxIndex;
  unsigned long lastCycleTime;
  unsigned long lastDisplayUpdate;
  bool autoCycle;
  bool needsRedraw;

  void begin() {
    Serial.println("Initializing display...");
    tft.begin();
    tft.setRotation(1); // Landscape

    // Initial splash screen
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(20, 100);
    tft.print("AxleWatch Receiver");
    tft.setTextSize(1);
    tft.setCursor(80, 130);
    tft.print("Initializing...");

    currentTxIndex = 0;
    lastCycleTime = millis();
    lastDisplayUpdate = millis();
    autoCycle = true;
    needsRedraw = true;

    Serial.println("Display initialized");
    delay(100); // Allow display to stabilize
  }

  void update(TransmitterData transmitters[], int numTx, GPSData* gpsData,
              AlarmManager* alarmMgr, bool wifiConnected, unsigned long uptimeMs) {

    // Auto-cycle through active transmitters
    if (autoCycle && millis() - lastCycleTime >= DISPLAY_CYCLE_INTERVAL) {
      cycleTx(transmitters, numTx);
      lastCycleTime = millis();
      needsRedraw = true;
    }

    // Limit display refresh rate to reduce flicker (update every 2 seconds or when needed)
    if (needsRedraw || (millis() - lastDisplayUpdate >= 2000)) {
      drawMainScreen(transmitters, numTx, gpsData, alarmMgr, wifiConnected, uptimeMs);
      lastDisplayUpdate = millis();
      needsRedraw = false;
    }
  }

  void cycleTx(TransmitterData transmitters[], int numTx) {
    int startIdx = currentTxIndex;
    do {
      currentTxIndex = (currentTxIndex + 1) % MAX_TRANSMITTERS;
      if (transmitters[currentTxIndex].active) break;
    } while (currentTxIndex != startIdx);
  }

  void pauseAutoCycle() {
    autoCycle = false;
  }

  void resumeAutoCycle() {
    autoCycle = true;
    lastCycleTime = millis();
  }

  void drawMainScreen(TransmitterData transmitters[], int numTx, GPSData* gpsData,
                      AlarmManager* alarmMgr, bool wifiConnected, unsigned long uptimeMs) {

    static bool firstDraw = true;
    if (firstDraw) {
      Serial.println(">>> First display update <<<");
      firstDraw = false;
    }

    tft.fillScreen(ILI9341_BLACK);

    // Header
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(5, 5);
    tft.print("AxleWatch");

    // WiFi status
    tft.setTextSize(1);
    tft.setCursor(250, 8);
    if (wifiConnected) {
      tft.setTextColor(ILI9341_GREEN);
      tft.print("WiFi OK");
    } else {
      tft.setTextColor(ILI9341_RED);
      tft.print("No WiFi");
    }

    // GPS status
    tft.setCursor(250, 18);
    if (gpsData->validFix) {
      tft.setTextColor(ILI9341_GREEN);
      tft.print("GPS OK");
    } else {
      tft.setTextColor(ILI9341_ORANGE);
      tft.print("GPS...");
    }

    // Active transmitter
    if (!transmitters[currentTxIndex].active) {
      tft.setTextSize(2);
      tft.setTextColor(ILI9341_YELLOW);
      tft.setCursor(60, 100);
      tft.print("No Active TX");
      drawButtons(alarmMgr);
      return;
    }

    TransmitterData* tx = &transmitters[currentTxIndex];

    // TX ID
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(5, 30);
    tft.print("TX: ");
    tft.print(tx->txID);

    // RSSI
    tft.setTextSize(1);
    tft.setCursor(200, 35);
    tft.print("RSSI:");
    tft.print(tx->rssi);
    tft.print("dBm");

    // Ambient temp
    tft.setCursor(5, 50);
    tft.setTextColor(ILI9341_CYAN);
    tft.print("Ambient: ");
    tft.print(tx->ambientTemp, 1);
    tft.print("C");

    // GPS Speed
    tft.setCursor(200, 50);
    tft.setTextColor(ILI9341_CYAN);
    tft.print("Speed:");
    tft.print(gpsData->speedKmh, 0);
    tft.print("km/h");

    // Temperature grid (3x3)
    int startY = 70;
    int rowHeight = 40;
    int colWidth = 106;

    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        int idx = row * 3 + col;
        int x = col * colWidth + 5;
        int y = startY + row * rowHeight;

        // Position label
        tft.setTextSize(1);
        tft.setTextColor(ILI9341_WHITE);
        tft.setCursor(x, y);
        tft.print("P");
        tft.print(idx + 1);
        tft.print(":");

        // Temperature value
        float temp = tx->temps[idx];
        uint16_t color = alarmMgr->getAlarmColor(tx->alarmLevels[idx]);

        tft.setTextSize(2);
        tft.setTextColor(color);
        tft.setCursor(x, y + 12);

        if (temp < 1.0) {
          tft.print("--");
        } else {
          if (temp < 100) tft.print(" ");
          tft.print(temp, 1);
        }
      }
    }

    // Draw control buttons
    drawButtons(alarmMgr);
  }

  void drawButtons(AlarmManager* alarmMgr) {
    int btnY = 210;

    // Mute button
    tft.fillRect(10, btnY, 90, 25, alarmMgr->alarmMuted ? ILI9341_DARKGREY : ILI9341_BLUE);
    tft.drawRect(10, btnY, 90, 25, ILI9341_WHITE);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(25, btnY + 8);
    tft.print("MUTE");

    // Acknowledge button
    tft.fillRect(120, btnY, 90, 25, alarmMgr->alarmAcknowledged ? ILI9341_DARKGREY : ILI9341_ORANGE);
    tft.drawRect(120, btnY, 90, 25, ILI9341_WHITE);
    tft.setCursor(135, btnY + 8);
    tft.print("ACK");

    // Settings button (placeholder)
    tft.fillRect(230, btnY, 80, 25, ILI9341_GREEN);
    tft.drawRect(230, btnY, 80, 25, ILI9341_WHITE);
    tft.setCursor(240, btnY + 8);
    tft.print("SETTINGS");
  }
};

// ======================== TOUCH INPUT ========================
class TouchInput {
public:
  unsigned long lastTouchTime;
  uint16_t lastRawX, lastRawY;
  static const unsigned long TOUCH_DEBOUNCE = 300; // 300ms debounce
  static const uint16_t TOUCH_THRESHOLD = 200; // Minimum pressure

  void begin() {
    ts.begin();
    ts.setRotation(1);
    lastTouchTime = 0;
    lastRawX = 0;
    lastRawY = 0;
    Serial.println("Touch initialized");
  }

  bool getTouchPoint(uint16_t* rawX, uint16_t* rawY, uint16_t* pressure) {
    if (!ts.touched()) return false;

    TS_Point p = ts.getPoint();
    *rawX = p.x;
    *rawY = p.y;
    *pressure = p.z;

    // Filter out noise and phantom touches
    // Reject if pressure is 0 (invalid) or below threshold
    if (*pressure == 0 || *pressure < TOUCH_THRESHOLD) return false;

    // Reject if coordinates are clearly invalid (0,0 or maxed out)
    if ((*rawX == 0 && *rawY == 0) || *rawX > 4095 || *rawY > 4095) return false;

    return true;
  }

  void handleTouch(DisplayManager* display, AlarmManager* alarmMgr) {
    uint16_t rawX, rawY, pressure;

    // Debounce - ignore touches too close together
    if (millis() - lastTouchTime < TOUCH_DEBOUNCE) return;

    if (!getTouchPoint(&rawX, &rawY, &pressure)) return;

    lastTouchTime = millis();
    lastRawX = rawX;
    lastRawY = rawY;

    // Map raw coordinates to screen coordinates (320x240) using calibrated values
    // NOTE: Coordinates appear inverted - swap and invert if needed
    int x = map(rawX, TOUCH_MIN_X, TOUCH_MAX_X, 320, 0);  // Inverted X
    int y = map(rawY, TOUCH_MIN_Y, TOUCH_MAX_Y, 240, 0);  // Inverted Y

    // Clamp to screen bounds
    x = constrain(x, 0, 319);
    y = constrain(y, 0, 239);

    Serial.print("Touch: X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.print(y);
    Serial.print(" (Raw: ");
    Serial.print(rawX);
    Serial.print(",");
    Serial.print(rawY);
    Serial.println(")");

    // Pause auto-cycle on any touch
    display->pauseAutoCycle();
    display->needsRedraw = true;

    // Check button regions (bottom of screen)
    int btnY = 210;

    // Mute button (10, btnY, 90x25)
    if (y >= btnY && y <= btnY + 25) {
      if (x >= 10 && x <= 100) {
        alarmMgr->muteAlarm();
        Serial.println(">>> Alarm MUTED");
        display->needsRedraw = true;
      }
      // Acknowledge button (120, btnY, 90x25)
      else if (x >= 120 && x <= 210) {
        alarmMgr->acknowledgeAlarm();
        Serial.println(">>> Alarm ACKNOWLEDGED");
        display->needsRedraw = true;
      }
      // Settings button (230, btnY, 80x25)
      else if (x >= 230 && x <= 310) {
        Serial.println(">>> Settings button pressed (not implemented)");
      }
    }
  }
};

// Forward declarations
class CloudUploadManager;
class GPSManager;
struct TransmitterData;

// WiFi state machine enum
enum WiFiState {
  WIFI_STATE_AP_MODE,
  WIFI_STATE_CONNECTING_STA,
  WIFI_STATE_CONNECTED_STA,
  WIFI_STATE_FAILED_STA
};

// Extern declarations for global variables (defined later in file)
extern GPSManager gpsManager;
extern TransmitterData transmitters[];
extern WiFiState wifiState;

// ======================== WEB CONFIG SERVER ========================
class WebConfigServer {
public:
  ConfigManager* configMgr;
  CloudUploadManager* cloudUpload;

  void begin(ConfigManager* cfg, CloudUploadManager* cloud) {
    configMgr = cfg;
    cloudUpload = cloud;
    setupRoutes();
  }

  void setupRoutes() {
    webServer.on("/", [this]() { handleRoot(); });
    webServer.on("/save", HTTP_POST, [this]() { handleSave(); });
    webServer.on("/status", [this]() { handleStatus(); });
    webServer.on("/live", [this]() { handleLive(); });
    webServer.on("/api/live", [this]() { handleApiLive(); });
    webServer.on("/api/config", [this]() { handleApiConfig(); });
    webServer.on("/api/upload/status", [this]() { handleUploadStatus(); });
    webServer.on("/wifi/status", [this]() { handleWifiStatus(); });
    webServer.begin();
    Serial.println("Web server started");
  }

  void handleClient() {
    webServer.handleClient();
  }

  void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>AxleWatch Config</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; margin: 20px; background: #f0f0f0; }
    .container { max-width: 600px; margin: auto; background: white; padding: 20px; border-radius: 8px; }
    h1 { color: #2196F3; }
    label { display: block; margin-top: 15px; font-weight: bold; }
    input, button { width: 100%; padding: 10px; margin-top: 5px; box-sizing: border-box; }
    button { background: #2196F3; color: white; border: none; cursor: pointer; font-size: 16px; }
    button:hover { background: #0b7dda; }
    .status { background: #e7f3ff; padding: 10px; border-left: 4px solid #2196F3; margin-bottom: 20px; }
  </style>
</head>
<body>
  <div class="container">
    <h1>AxleWatch Receiver Configuration</h1>
    <div class="status">
      <strong>Device ID:</strong> )rawliteral" + String(configMgr->config.deviceID) + R"rawliteral(<br>
      <strong>Status:</strong> Configuration Mode<br>
      <strong><a href="/live" style="color: #2196F3;">View Live Dashboard →</a></strong>
    </div>
    <form action="/save" method="POST">
      <label>WiFi SSID:</label>
      <input type="text" name="ssid" value=")rawliteral" + String(configMgr->config.wifiSSID) + R"rawliteral(" required>

      <label>WiFi Password:</label>
      <input type="password" name="password" value=")rawliteral" + String(configMgr->config.wifiPassword) + R"rawliteral(">

      <label>Cloud Endpoint URL:</label>
      <input type="text" name="endpoint" value=")rawliteral" + String(configMgr->config.cloudEndpoint) + R"rawliteral(">

      <label>Cloud API Key:</label>
      <input type="password" name="apiKey" value=")rawliteral" + String(configMgr->config.apiKey) + R"rawliteral(" placeholder="Enter API key from axlewatch.com">

      <label>Warning Threshold (°C above ambient):</label>
      <input type="number" step="0.1" name="warnOffset" value=")rawliteral" + String(configMgr->config.warnOffset) + R"rawliteral(">

      <label>Critical Threshold (°C above ambient):</label>
      <input type="number" step="0.1" name="critOffset" value=")rawliteral" + String(configMgr->config.critOffset) + R"rawliteral(">

      <label>
        <input type="checkbox" name="cloudEnabled" value="1" )rawliteral" + String(configMgr->config.cloudEnabled ? "checked" : "") + R"rawliteral(>
        Enable Cloud Upload
      </label>

      <button type="submit">Save Configuration</button>
    </form>
  </div>
</body>
</html>
)rawliteral";

    webServer.send(200, "text/html", html);
  }

  void handleSave() {
    if (webServer.hasArg("ssid")) {
      strncpy(configMgr->config.wifiSSID, webServer.arg("ssid").c_str(),
              sizeof(configMgr->config.wifiSSID) - 1);
    }

    if (webServer.hasArg("password")) {
      strncpy(configMgr->config.wifiPassword, webServer.arg("password").c_str(),
              sizeof(configMgr->config.wifiPassword) - 1);
    }

    if (webServer.hasArg("endpoint")) {
      strncpy(configMgr->config.cloudEndpoint, webServer.arg("endpoint").c_str(),
              sizeof(configMgr->config.cloudEndpoint) - 1);
    }

    if (webServer.hasArg("apiKey")) {
      strncpy(configMgr->config.apiKey, webServer.arg("apiKey").c_str(),
              sizeof(configMgr->config.apiKey) - 1);
    }

    if (webServer.hasArg("warnOffset")) {
      configMgr->config.warnOffset = webServer.arg("warnOffset").toFloat();
    }

    if (webServer.hasArg("critOffset")) {
      configMgr->config.critOffset = webServer.arg("critOffset").toFloat();
    }

    configMgr->config.cloudEnabled = webServer.hasArg("cloudEnabled");

    configMgr->saveConfig();

    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Saved</title>
  <meta http-equiv="refresh" content="3;url=/">
  <style>
    body { font-family: Arial; text-align: center; margin-top: 50px; }
    .success { color: green; font-size: 24px; }
  </style>
</head>
<body>
  <div class="success">✓ Configuration Saved!</div>
  <p>Redirecting...</p>
</body>
</html>
)rawliteral";

    webServer.send(200, "text/html", html);
  }

  void handleStatus() {
    StaticJsonDocument<512> doc;
    doc["deviceID"] = configMgr->config.deviceID;
    doc["wifiSSID"] = configMgr->config.wifiSSID;
    doc["cloudEnabled"] = configMgr->config.cloudEnabled;
    doc["warnOffset"] = configMgr->config.warnOffset;
    doc["critOffset"] = configMgr->config.critOffset;
    doc["uptime"] = millis();

    String json;
    serializeJson(doc, json);
    webServer.send(200, "application/json", json);
  }

  void handleUploadStatus(); // Defined after CloudUploadManager
  void handleLive();         // Live dashboard page
  void handleApiLive();      // Live data JSON API
  void handleApiConfig();    // Configuration JSON API
  void handleWifiStatus();   // WiFi status JSON API
};

// ======================== CLOUD UPLOAD MANAGER ========================
class CloudUploadManager {
public:
  unsigned long lastUploadTime;
  bool uploadInProgress;
  ConfigManager* configMgr;

  void begin(ConfigManager* cfg) {
    configMgr = cfg;
    lastUploadTime = 0;
    uploadInProgress = false;
  }

  void update(TransmitterData transmitters[], int numTx, GPSData* gpsData,
              AlarmManager* alarmMgr, bool wifiConnected) {

    if (!configMgr->config.cloudEnabled) return;
    if (!wifiConnected) return;
    if (uploadInProgress) return;

    if (millis() - lastUploadTime >= CLOUD_UPLOAD_INTERVAL) {
      uploadData(transmitters, numTx, gpsData, alarmMgr);
      lastUploadTime = millis();
    }
  }

  // Extract numeric trailer ID from transmitter ID string
  // Examples: "TRAILER1" → 1, "DOLLY3" → 3, "TX001" → 1
  int extractTrailerId(const char* txID) {
    if (strncmp(txID, "TRAILER", 7) == 0) {
      return atoi(txID + 7);
    } else if (strncmp(txID, "DOLLY", 5) == 0) {
      return atoi(txID + 5);
    } else if (strncmp(txID, "TX", 2) == 0) {
      return atoi(txID + 2);
    }
    return 0; // Unknown format
  }

  void uploadData(TransmitterData transmitters[], int numTx, GPSData* gpsData,
                  AlarmManager* alarmMgr) {

    uploadInProgress = true;

    Serial.println("[CloudUpload] Starting upload...");

    // Check if API key is configured
    if (strlen(configMgr->config.apiKey) == 0) {
      Serial.println("[CloudUpload] ERROR: API key not configured");
      uploadInProgress = false;
      return;
    }

    int activeTxCount = 0;
    bool anySuccess = false;

    // Send separate request for each active trailer (per CLOUD_SETUP spec)
    for (int i = 0; i < MAX_TRANSMITTERS; i++) {
      if (!transmitters[i].active) continue;

      activeTxCount++;
      Serial.printf("[CloudUpload] Uploading trailer %s...\n", transmitters[i].txID);

      // Build JSON payload according to CLOUD_SETUP spec
      StaticJsonDocument<1024> doc;

      // REQUIRED: identify which receiver sent this data
      doc["device_id"] = configMgr->config.deviceID;

      // ISO 8601 timestamp (simplified - using millis as placeholder)
      // In production, sync with NTP or GPS time
      char timestamp[32];
      unsigned long seconds = millis() / 1000;
      snprintf(timestamp, sizeof(timestamp), "2025-01-01T%02lu:%02lu:%02luZ",
               (seconds / 3600) % 24, (seconds / 60) % 60, seconds % 60);
      doc["timestamp"] = timestamp;

      // Trailer ID
      doc["trailer_id"] = transmitters[i].txID;

      // Hub temperature readings (hub_1 through hub_8)
      // TX supports 9 sensors, web dashboard displays 8 hubs
      JsonObject readings = doc.createNestedObject("readings");
      readings["hub_1"] = transmitters[i].temps[0];
      readings["hub_2"] = transmitters[i].temps[1];
      readings["hub_3"] = transmitters[i].temps[2];
      readings["hub_4"] = transmitters[i].temps[3];
      readings["hub_5"] = transmitters[i].temps[4];
      readings["hub_6"] = transmitters[i].temps[5];
      readings["hub_7"] = transmitters[i].temps[6];
      readings["hub_8"] = transmitters[i].temps[7];
      readings["ambient_temp"] = transmitters[i].ambientTemp;

      // Location data (from GPS)
      JsonObject location = doc.createNestedObject("location");
      location["latitude"] = gpsData->latitude;
      location["longitude"] = gpsData->longitude;
      location["speed"] = gpsData->speedKmh;

      // Alert (if any hub temperature delta exceeds threshold)
      // Compare delta (temp - ambient) to thresholds, not absolute temperature
      float ambient = transmitters[i].ambientTemp;
      float maxDelta = 0;
      float maxTemp = 0;
      for (int j = 0; j < 8; j++) {
        float temp = transmitters[i].temps[j];
        if (temp > 1.0) { // Ignore unused sensors (0.0)
          float delta = temp - ambient;
          if (delta > maxDelta) {
            maxDelta = delta;
            maxTemp = temp;
          }
        }
      }

      if (maxDelta > configMgr->config.warnOffset) {
        JsonObject alert = doc.createNestedObject("alert");
        if (maxDelta > configMgr->config.critOffset) {
          alert["level"] = "critical";
        } else {
          alert["level"] = "warning";
        }
        char msg[100];
        snprintf(msg, sizeof(msg), "High temperature detected: %.1f°C (%.1f°C above ambient)", maxTemp, maxDelta);
        alert["message"] = msg;
      }

      String payload;
      serializeJson(doc, payload);

      Serial.printf("[CloudUpload] Trailer %s: RSSI=%d, Ambient=%.1f°C, MaxHub=%.1f°C\n",
                    transmitters[i].txID, transmitters[i].rssi,
                    transmitters[i].ambientTemp, maxTemp);
      Serial.printf("[CloudUpload] Payload size: %d bytes\n", payload.length());
      Serial.printf("[CloudUpload] Posting to: %s\n", configMgr->config.cloudEndpoint);

      // Send HTTP POST with manual redirect handling
      HTTPClient http;
      http.setTimeout(10000); // 10 second timeout

      String currentUrl = configMgr->config.cloudEndpoint;
      int httpCode = -1;
      int redirectCount = 0;
      const int MAX_REDIRECTS = 3;

      // Build Authorization header
      String authHeader = "Bearer ";
      authHeader += configMgr->config.apiKey;

      // Manual redirect loop (HTTPClient doesn't follow POST redirects properly)
      while (redirectCount <= MAX_REDIRECTS) {
        http.begin(currentUrl);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("Authorization", authHeader);

        httpCode = http.POST(payload);

        if (httpCode > 0) {
          Serial.printf("[CloudUpload] HTTP Response code: %d\n", httpCode);

          // Check if this is a redirect (3xx status)
          if (httpCode >= 300 && httpCode < 400) {
            String location = http.getLocation();

            if (location.length() > 0) {
              Serial.printf("[CloudUpload] Redirected to: %s\n", location.c_str());
              http.end(); // Close current connection
              currentUrl = location;
              redirectCount++;
              continue; // Retry with new URL
            } else {
              Serial.println("[CloudUpload] Redirect response but no Location header");
              break;
            }
          }

          // Not a redirect, process the response
          break;
        } else {
          Serial.printf("[CloudUpload] Connection error: %s\n", http.errorToString(httpCode).c_str());
          break;
        }
      }

      // Process final response
      if (httpCode > 0) {
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED) {
          uploadCount++;
          lastSuccess = true;
          anySuccess = true;
          lastHttpStatus = httpCode;
          Serial.printf("[CloudUpload] ✓ Trailer %s uploaded successfully\n", transmitters[i].txID);
        } else {
          lastSuccess = false;
          lastHttpStatus = httpCode;
          String response = http.getString();
          Serial.printf("[CloudUpload] ✗ Trailer %s upload failed (HTTP %d)\n",
                        transmitters[i].txID, httpCode);
          if (response.length() > 0 && response.length() < 500) {
            Serial.printf("[CloudUpload] Response: %s\n", response.c_str());
          }
        }
      } else {
        lastSuccess = false;
        lastHttpStatus = httpCode;
      }

      http.end();
    }

    Serial.printf("[CloudUpload] Finished. %d trailers processed. Overall success: %s\n",
                  activeTxCount, anySuccess ? "YES" : "NO");
    uploadInProgress = false;
  }

  // Upload statistics (for /api/upload/status endpoint)
  unsigned long uploadCount = 0;
  bool lastSuccess = false;
  int lastHttpStatus = 0;
};

// ======================== WEB SERVER METHODS (after CloudUploadManager) ========================
void WebConfigServer::handleUploadStatus() {
  StaticJsonDocument<256> doc;
  doc["enabled"] = configMgr->config.cloudEnabled;
  doc["lastUploadTime"] = cloudUpload->lastUploadTime / 1000; // Convert to seconds
  doc["lastSuccess"] = cloudUpload->lastSuccess;
  doc["lastHttpStatus"] = cloudUpload->lastHttpStatus;
  doc["uploadCount"] = cloudUpload->uploadCount;

  unsigned long secondsSince = (millis() - cloudUpload->lastUploadTime) / 1000;
  doc["secondsSinceLastUpload"] = secondsSince;

  String json;
  serializeJson(doc, json);
  webServer.send(200, "application/json", json);
}

void WebConfigServer::handleLive() {
  // Live dashboard page with real-time data display
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>AxleWatch Live</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="5">
  <style>
    body { font-family: Arial; margin: 20px; background: #0b1220; color: #e8eefc; }
    .container { max-width: 800px; margin: auto; }
    h1 { color: #4cc9f0; }
    .card { background: #141b2d; border: 1px solid #334; border-radius: 10px; padding: 20px; margin: 16px 0; }
    .card h2 { color: #4cc9f0; margin: 0 0 16px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 12px; }
    .sensor-box { background: #1a2332; border: 2px solid #445; border-radius: 8px; padding: 16px; text-align: center; }
    .sensor-box h3 { font-size: 14px; margin: 0 0 8px; opacity: .9; color: #4cc9f0; }
    .sensor-box .temp { font-size: 32px; font-weight: bold; color: #7bd88f; }
    .status-row { display: flex; justify-content: space-between; padding: 8px 0; border-bottom: 1px solid #334; }
    .status-label { color: #888; font-size: 14px; }
    .status-value { color: #4cc9f0; font-weight: 600; font-size: 14px; }
    a { color: #4cc9f0; text-decoration: none; }
  </style>
</head>
<body>
  <div class="container">
    <h1>AxleWatch Live Dashboard</h1>
    <p><a href="/">← Back to Configuration</a></p>
    <div class="card">
      <h2>Active Transmitters</h2>
      <div id="transmitters"></div>
    </div>
    <div class="card">
      <h2>System Status</h2>
      <div class="status-row">
        <span class="status-label">Device ID:</span>
        <span class="status-value" id="deviceId">-</span>
      </div>
      <div class="status-row">
        <span class="status-label">WiFi:</span>
        <span class="status-value" id="wifiStatus">-</span>
      </div>
      <div class="status-row">
        <span class="status-label">GPS:</span>
        <span class="status-value" id="gpsStatus">-</span>
      </div>
    </div>
  </div>
  <script>
  fetch('/api/live').then(r=>r.json()).then(d=>{
    document.getElementById('deviceId').textContent=d.deviceId;
    document.getElementById('wifiStatus').textContent=d.wifiConnected?'Connected':'Disconnected';
    document.getElementById('gpsStatus').textContent=d.gps.fix?'Fix ('+d.gps.satellites+' sats)':'No Fix';
    let html='';
    d.trailers.forEach(t=>{
      html+='<h3>'+t.id+'</h3><div class="grid">';
      t.hubTemperatures.forEach((temp,i)=>{
        html+='<div class="sensor-box"><h3>Hub '+(i+1)+'</h3><div class="temp">'+temp.toFixed(1)+'°C</div></div>';
      });
      html+='</div><p>Ambient: '+t.ambientTemp.toFixed(1)+'°C | RSSI: '+t.rssi+'</p>';
    });
    document.getElementById('transmitters').innerHTML=html||'<p>No active transmitters</p>';
  });
  </script>
</body>
</html>
)rawliteral";
  webServer.send(200, "text/html", html);
}

void WebConfigServer::handleApiLive() {
  // Return live data in JSON format matching cloud upload structure
  DynamicJsonDocument doc(4096);

  doc["deviceId"] = configMgr->config.deviceID;
  doc["timestamp"] = millis() / 1000;
  doc["wifiConnected"] = (wifiState == WIFI_STATE_CONNECTED_STA);

  // GPS data
  JsonObject gpsObj = doc.createNestedObject("gps");
  GPSData* gpsData = gpsManager.getData();
  gpsObj["valid"] = gpsData->validFix;
  gpsObj["latitude"] = gpsData->latitude;
  gpsObj["longitude"] = gpsData->longitude;
  gpsObj["speed"] = gpsData->speedKmh;
  gpsObj["course"] = gpsData->course;
  gpsObj["satellites"] = gpsData->satellites;
  gpsObj["fix"] = gpsData->validFix;

  // Trailers
  JsonArray trailersArray = doc.createNestedArray("trailers");
  for (int i = 0; i < MAX_TRANSMITTERS; i++) {
    if (!transmitters[i].active) continue;

    JsonObject trailer = trailersArray.createNestedObject();
    trailer["id"] = i + 1;
    trailer["name"] = transmitters[i].txID;
    trailer["online"] = true;
    trailer["rssi"] = transmitters[i].rssi;
    trailer["lastUpdate"] = transmitters[i].lastReceived / 1000;
    trailer["ambientTemp"] = transmitters[i].ambientTemp;

    JsonArray hubTemps = trailer.createNestedArray("hubTemperatures");
    for (int j = 0; j < 8 && j < NUM_TEMP_SENSORS; j++) {
      hubTemps.add(transmitters[i].temps[j]);
    }
  }

  doc["wifiRssi"] = WiFi.RSSI();

  String json;
  serializeJson(doc, json);
  webServer.send(200, "application/json", json);
}

void WebConfigServer::handleApiConfig() {
  // Return current configuration
  StaticJsonDocument<512> doc;
  doc["deviceID"] = configMgr->config.deviceID;
  doc["wifiSSID"] = configMgr->config.wifiSSID;
  doc["cloudEndpoint"] = configMgr->config.cloudEndpoint;
  doc["cloudEnabled"] = configMgr->config.cloudEnabled;
  doc["warnOffset"] = configMgr->config.warnOffset;
  doc["critOffset"] = configMgr->config.critOffset;
  doc["uptime"] = millis() / 1000;

  String json;
  serializeJson(doc, json);
  webServer.send(200, "application/json", json);
}

void WebConfigServer::handleWifiStatus() {
  // Return WiFi connection status
  StaticJsonDocument<256> doc;
  doc["connected"] = (wifiState == WIFI_STATE_CONNECTED_STA);
  doc["ssid"] = configMgr->config.wifiSSID;
  doc["rssi"] = WiFi.RSSI();
  doc["ip"] = WiFi.localIP().toString();
  doc["apIP"] = WiFi.softAPIP().toString();
  doc["mode"] = (wifiState == WIFI_STATE_AP_MODE) ? "AP" :
                (wifiState == WIFI_STATE_CONNECTED_STA) ? "STA" : "Connecting";

  String json;
  serializeJson(doc, json);
  webServer.send(200, "application/json", json);
}

// ======================== GLOBAL INSTANCES ========================
ConfigManager configManager;
LoRaManager loraManager;
GPSManager gpsManager;
AlarmManager alarmManager;
SDLogger sdLogger;
DisplayManager displayManager;
TouchInput touchInput;
WebConfigServer webConfigServer;
CloudUploadManager cloudUploadManager;

TransmitterData transmitters[MAX_TRANSMITTERS];
int numActiveTransmitters = 0;

// WiFi state machine (enum defined earlier before WebConfigServer class)
WiFiState wifiState = WIFI_STATE_AP_MODE;
unsigned long wifiStateChangeTime = 0;
unsigned long apModeStartTime = 0;
bool apModeComplete = false;

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== AxleWatch Receiver v1.0 ===");
  Serial.println("Hardware: ESP32-S3 with 2 SPI buses");
  Serial.println("  SPI (remapped to pins 35,36,37): Display + Touch + SD Card");
  Serial.println("  HSPI (pins 11,12,13): LoRa module");

  // Initialize LED pins (only 2 LEDs on board)
  Serial.println("Initializing LEDs...");
  pinMode(LED1, OUTPUT);  // System status
  pinMode(LED2, OUTPUT);  // GPS/Activity status
  digitalWrite(LED1, HIGH);  // Turn on system LED during init
  digitalWrite(LED2, LOW);

  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Configure touch IRQ pin with pullup to prevent phantom touches
  pinMode(TOUCH_IRQ, INPUT_PULLUP);

  // CRITICAL: Initialize default SPI bus with SD CARD PINS (not ESP32 default pins!)
  // Per working code: Display + Touch + SD all share SPI on pins 35, 36, 37
  Serial.println("\nInitializing default SPI bus (Display + Touch + SD)...");
  Serial.print("  Using SD card pins: SCK=");
  Serial.print(SD_SCK_PIN);
  Serial.print(", MISO=");
  Serial.print(SD_MISO_PIN);
  Serial.print(", MOSI=");
  Serial.println(SD_MOSI_PIN);

  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN);  // Remap SPI to SD card pins!

  // Set all CS pins HIGH (deselected) before device initialization
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  Serial.println("  Default SPI ready (3 devices: Display, Touch, SD)");

  delay(100);

  // Initialize configuration
  configManager.begin();
  Serial.print("Device ID: ");
  Serial.println(configManager.getDeviceID());

  // Initialize display
  displayManager.begin();

  // Initialize touch
  touchInput.begin();

  // Initialize LoRa
  loraManager.begin();

  // Initialize GPS
  gpsManager.begin();

  // Initialize alarm manager
  alarmManager.begin();

  // Initialize SD logger
  sdLogger.begin();

  // Initialize all transmitter slots
  for (int i = 0; i < MAX_TRANSMITTERS; i++) {
    transmitters[i].active = false;
    transmitters[i].lastReceived = 0;
  }

  // Initialize cloud upload manager
  cloudUploadManager.begin(&configManager);

  // Start AP mode (mandatory 60 seconds on boot)
  Serial.println("Starting AP mode for 60 seconds...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  webConfigServer.begin(&configManager, &cloudUploadManager);

  apModeStartTime = millis();
  wifiState = WIFI_STATE_AP_MODE;
  wifiStateChangeTime = millis();

  Serial.println("Setup complete. AP mode active.");
  Serial.println("Connect to: " + String(AP_SSID));
  Serial.println("Password: " + String(AP_PASSWORD));
  Serial.println("Config portal: http://192.168.4.1");

  // Show splash screen for 2 seconds
  delay(2000);

  // Force first display update
  displayManager.needsRedraw = true;

  digitalWrite(LED1, LOW);
  Serial.println("\n>>> Entering main loop <<<\n");
}

// ======================== LOOP ========================
void loop() {
  unsigned long now = millis();

  // Handle WiFi state machine
  handleWiFiStateMachine(now);

  // Update GPS
  gpsManager.update();

  // Receive LoRa packets
  handleLoRaReception();

  // Update alarms
  alarmManager.update(transmitters, MAX_TRANSMITTERS,
                     configManager.config.warnOffset,
                     configManager.config.critOffset);
  alarmManager.resetAlarm();

  // Handle touch input
  touchInput.handleTouch(&displayManager, &alarmManager);

  // Update display
  bool wifiConnected = (wifiState == WIFI_STATE_CONNECTED_STA);
  displayManager.update(transmitters, MAX_TRANSMITTERS, gpsManager.getData(),
                       &alarmManager, wifiConnected, now);

  // Cloud upload (only when WiFi connected)
  if (apModeComplete && wifiConnected) {
    cloudUploadManager.update(transmitters, MAX_TRANSMITTERS, gpsManager.getData(),
                             &alarmManager, wifiConnected);
  }

  // Web server
  webConfigServer.handleClient();

  // Cleanup inactive transmitters (no packet for 90 seconds)
  cleanupInactiveTransmitters(now);

  // Status LED indicators (only 2 LEDs available)
  static unsigned long lastBlink = 0;
  if (now - lastBlink >= 1000) {
    // LED1: System heartbeat - blinks to show system is running
    digitalWrite(LED1, !digitalRead(LED1));

    // LED2: GPS status - solid when locked, blink when searching
    // (LoRa activity flash handled in handleLoRaReception)
    digitalWrite(LED2, gpsManager.getData()->validFix ? HIGH : !digitalRead(LED2));

    lastBlink = now;
  }
}

// ======================== WIFI STATE MACHINE ========================
void handleWiFiStateMachine(unsigned long now) {
  static unsigned long staRetryTime = 0;
  static int staRetryAttempts = 0;

  switch (wifiState) {
    case WIFI_STATE_AP_MODE:
      // Stay in AP mode for exactly 60 seconds
      if (now - apModeStartTime >= AP_MODE_DURATION) {
        Serial.println("AP mode timeout. Transitioning to STA mode...");
        apModeComplete = true;

        // Check if we have WiFi credentials
        if (strlen(configManager.config.wifiSSID) > 0) {
          // Switch to STA mode
          WiFi.mode(WIFI_AP_STA); // Keep AP running
          WiFi.begin(configManager.config.wifiSSID, configManager.config.wifiPassword);

          Serial.print("Connecting to WiFi: ");
          Serial.println(configManager.config.wifiSSID);

          wifiState = WIFI_STATE_CONNECTING_STA;
          wifiStateChangeTime = now;
          staRetryAttempts = 0;
        } else {
          Serial.println("No WiFi credentials configured. Staying in AP mode.");
          wifiState = WIFI_STATE_FAILED_STA;
        }
      }
      break;

    case WIFI_STATE_CONNECTING_STA:
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());

        wifiState = WIFI_STATE_CONNECTED_STA;
        wifiStateChangeTime = now;
        staRetryAttempts = 0;
      } else if (now - wifiStateChangeTime >= 15000) {
        // Connection timeout after 15 seconds
        Serial.println("\nWiFi connection timeout");
        wifiState = WIFI_STATE_FAILED_STA;
        wifiStateChangeTime = now;
        staRetryTime = now + (5000 * (1 << min(staRetryAttempts, 5))); // Exponential backoff
        staRetryAttempts++;
      }
      break;

    case WIFI_STATE_CONNECTED_STA:
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected!");
        wifiState = WIFI_STATE_FAILED_STA;
        wifiStateChangeTime = now;
        staRetryTime = now + 5000;
      }
      break;

    case WIFI_STATE_FAILED_STA:
      // Retry connection with exponential backoff
      if (now >= staRetryTime && strlen(configManager.config.wifiSSID) > 0) {
        Serial.println("Retrying WiFi connection...");
        WiFi.begin(configManager.config.wifiSSID, configManager.config.wifiPassword);
        wifiState = WIFI_STATE_CONNECTING_STA;
        wifiStateChangeTime = now;
      }
      break;
  }
}

// ======================== LORA RECEPTION ========================
void handleLoRaReception() {
  static unsigned long loraLedOffTime = 0;
  char packet[128];
  int rssi;

  if (loraManager.receivePacket(packet, sizeof(packet), &rssi)) {
    // Brief flash on LED2 for LoRa activity (only 2 LEDs available)
    digitalWrite(LED2, HIGH);
    loraLedOffTime = millis() + 100; // Keep LED on for 100ms

    Serial.print("LoRa RX: ");
    Serial.print(packet);
    Serial.print(" (RSSI: ");
    Serial.print(rssi);
    Serial.println(")");

    TransmitterData
Filter your search...
Type:

All
Topic:

All






Filter your search...
Type:

All
Topic:

All





 tempTx;
    if (loraManager.parsePacket(packet, &tempTx)) {
      tempTx.rssi = rssi;
Filter your search...
Type:

All
Topic:

All





177117721773177417751776177717781779178017811782178317841785178617871788178917901791179217931794179517961797179817991800180118021803180418051806180718081809
        transmitters[i].lastReceived <= now &&
        (now - transmitters[i].lastReceived > INACTIVE_TIMEOUT)) {
      Serial.print("Transmitter ");
      Serial.print(transmitters[i].txID);
      Serial.println(" timed out");
      transmitters[i].active = false;
    }
  }
}


      // Find or create transmitter slot
      int txSlot = findOrCreateTransmitter(tempTx.txID);
      if (txSlot >= 0) {
        transmitters[txSlot] = tempTx;

        // Log to SD card
        sdLogger.logData(&transmitters[txSlot], gpsManager.getData(), rssi);

        Serial.print("Updated TX slot ");
        Serial.print(txSlot);
        Serial.print(": ");
        Serial.println(tempTx.txID);
      }
    } else {
      Serial.println("Invalid packet format");
    }
  }

  // Turn off LED after flash duration (will be controlled by GPS status in main loop)
  if (loraLedOffTime > 0 && millis() >= loraLedOffTime) {
    loraLedOffTime = 0;
    // Don't turn off LED here - let the main loop handle LED2 state
  }
}

// ======================== TRANSMITTER MANAGEMENT ========================
int findOrCreateTransmitter(const char* txID) {
  // Find existing transmitter
  for (int i = 0; i < MAX_TRANSMITTERS; i++) {
    if (transmitters[i].active && strcmp(transmitters[i].txID, txID) == 0) {
      return i;
    }
  }

  // Find empty slot
  for (int i = 0; i < MAX_TRANSMITTERS; i++) {
    if (!transmitters[i].active) {
      Serial.print("New transmitter in slot ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(txID);
      return i;
    }
  }

  Serial.println("No free transmitter slots!");
  return -1;
}

void cleanupInactiveTransmitters(unsigned long now) {
  const unsigned long INACTIVE_TIMEOUT = 90000; // 90 seconds (allow for slow transmit rates)

  for (int i = 0; i < MAX_TRANSMITTERS; i++) {
    // Check if active and lastReceived is in the past (prevent unsigned underflow)
    if (transmitters[i].active &&
        transmitters[i].lastReceived <= now &&
        (now - transmitters[i].lastReceived > INACTIVE_TIMEOUT)) {
      Serial.print("Transmitter ");
      Serial.print(transmitters[i].txID);
      Serial.println(" timed out");
      transmitters[i].active = false;
    }
  }
}
