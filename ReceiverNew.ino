/*
 * AxleWatch Receiver Firmware v2.0
 * ESP32-WROOM-32 Homebrew Hardware
 *
 * Simplified version for non-touch 1.9" ST7789 display
 * with physical button input
 *
 * Features:
 * - LoRa 433MHz reception from trailer transmitters
 * - 1.9" ST7789 TFT display (170x320) with simple UI
 * - GPS integration for location/speed
 * - SD card logging
 * - LED status indicators (Green/Yellow/Red)
 * - Piezo buzzer alerts
 * - Physical button for alarm acknowledge/UI navigation
 *
 * Hardware: ESP32-WROOM-32, ST7789, RA-01 LoRa, GY-GPS6MV2, V474 SD
 */

// ======================== INCLUDES ========================
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <FS.h>

// ======================== PIN DEFINITIONS ========================
// ESP32-WROOM-32 Pin Assignments - AxleWatch Homebrew Receiver

// ============= DISPLAY PINS (VSPI) =============
#define TFT_CS          15
#define TFT_DC          16
#define TFT_RST         4
#define TFT_MOSI        23
#define TFT_SCK         18
// NOTE: TFT Backlight - tie directly to 3.3V (no GPIO control needed)

// ============= LORA PINS (HSPI) =============
#define LORA_CS         5
#define LORA_RST        33
#define LORA_DIO0       26
#define LORA_MOSI       13
#define LORA_MISO       12
#define LORA_SCK        14

// ============= SD CARD PINS (VSPI - shared with TFT) =============
#define SD_CS           27
// SD uses shared VSPI: MOSI=23, MISO=19, SCK=18

// ============= GPS PINS (UART2) =============
#define GPS_RX          34    // ESP32 RX <- GPS TX
#define GPS_TX          32    // ESP32 TX -> GPS RX

// ============= LED PINS (Active HIGH - GPIO -> resistor -> LED -> GND) =============
#define LED_GREEN       21
#define LED_YELLOW      22
#define LED_RED         17

// ============= BUZZER PIN =============
#define BUZZER_PIN      25

// ============= BUTTON PIN =============
#define BUTTON_PIN      35    // Requires external 10k pull-up

// ======================== CONSTANTS ========================
#define LORA_FREQUENCY      433E6
#define LORA_SF             7
#define LORA_BW             125E3

#define NUM_TEMP_SENSORS    9
#define MAX_TRANSMITTERS    10

#define DEFAULT_WARN_OFFSET     40.0  // Degrees above ambient for warning
#define DEFAULT_CRIT_OFFSET     60.0  // Degrees above ambient for critical

#define DISPLAY_CYCLE_INTERVAL  5000  // 5 seconds between TX cycling
#define GPS_UPDATE_INTERVAL     1000  // 1 second
#define TX_TIMEOUT              30000 // 30 seconds before TX marked inactive

// Screen dimensions (ST7789 1.9" in landscape)
#define SCREEN_WIDTH    320
#define SCREEN_HEIGHT   170

// ======================== GLOBAL OBJECTS ========================
// SPI Buses
SPIClass hspi(HSPI);  // Dedicated for LoRa

// Display (uses default VSPI)
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

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
  int satellites;
  bool validFix;
};

// ======================== GLOBAL VARIABLES ========================
TransmitterData transmitters[MAX_TRANSMITTERS];
GPSData gpsData;

int currentTxIndex = 0;
int activeTxCount = 0;

// Alarm state
bool alarmActive = false;
bool alarmMuted = false;
uint8_t currentAlarmLevel = 0;  // 0=OK, 1=WARN, 2=CRIT

// Timing
unsigned long lastDisplayCycle = 0;
unsigned long lastGPSUpdate = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastBuzzerToggle = 0;
bool buzzerState = false;

// Button state
bool lastButtonState = HIGH;
unsigned long lastButtonPress = 0;
unsigned long buttonPressStart = 0;
bool buttonHeld = false;

// SD card
bool sdAvailable = false;

// Display state
enum DisplayMode {
  MODE_OVERVIEW,    // Shows current TX temps
  MODE_DETAIL,      // Shows all 9 positions for current TX
  MODE_STATUS       // Shows system status (GPS, SD, etc)
};
DisplayMode displayMode = MODE_OVERVIEW;

// ======================== FUNCTION PROTOTYPES ========================
void initLoRa();
void initGPS();
void initDisplay();
void initSD();
void initLEDs();
void initBuzzer();
void initButton();

void updateLoRa();
void updateGPS();
void updateDisplay();
void updateAlarms();
void updateLEDs();
void updateBuzzer();
void handleButton();

bool parseLoRaPacket(const char* packet, TransmitterData* tx);
void logToSD(TransmitterData* tx);
void cycleTx();

void drawOverviewScreen();
void drawDetailScreen();
void drawStatusScreen();
void drawStatusBar();

uint16_t getAlarmColor(uint8_t level);
int countActiveTx();

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("================================");
  Serial.println("  AxleWatch Receiver v2.0");
  Serial.println("  Homebrew ESP32 Hardware");
  Serial.println("================================");
  Serial.println();

  // Initialize all subsystems
  initLEDs();
  initButton();
  initBuzzer();

  // IMPORTANT: Initialize SD before display (shared SPI bus)
  initSD();
  initDisplay();

  initLoRa();
  initGPS();

  // Clear transmitter data
  for (int i = 0; i < MAX_TRANSMITTERS; i++) {
    transmitters[i].active = false;
    memset(transmitters[i].txID, 0, sizeof(transmitters[i].txID));
  }

  Serial.println();
  Serial.println("Initialization complete!");
  Serial.println("Waiting for LoRa packets...");
  Serial.println();

  // Initial display
  drawOverviewScreen();
}

// ======================== MAIN LOOP ========================
void loop() {
  // Process incoming LoRa packets
  updateLoRa();

  // Update GPS data
  updateGPS();

  // Handle button input
  handleButton();

  // Update alarm states
  updateAlarms();

  // Update LED indicators
  updateLEDs();

  // Update buzzer
  updateBuzzer();

  // Update display (rate limited)
  updateDisplay();

  // Mark stale transmitters as inactive
  for (int i = 0; i < MAX_TRANSMITTERS; i++) {
    if (transmitters[i].active &&
        (millis() - transmitters[i].lastReceived > TX_TIMEOUT)) {
      transmitters[i].active = false;
      Serial.print("TX ");
      Serial.print(transmitters[i].txID);
      Serial.println(" marked inactive (timeout)");
    }
  }
}

// ======================== INITIALIZATION FUNCTIONS ========================

void initLEDs() {
  Serial.println("Initializing LEDs...");
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  // LEDs are active HIGH - turn all off
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);

  // Quick LED test
  digitalWrite(LED_RED, HIGH);
  delay(200);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, HIGH);
  delay(200);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, HIGH);
  delay(200);
  digitalWrite(LED_GREEN, LOW);  // All off after test

  Serial.println("  LEDs OK");
}

void initButton() {
  Serial.println("Initializing button...");
  pinMode(BUTTON_PIN, INPUT);  // External pull-up required
  Serial.println("  Button OK (GPIO 35)");
}

void initBuzzer() {
  Serial.println("Initializing buzzer...");
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Quick beep test
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("  Buzzer OK");
}

void initSD() {
  Serial.println("Initializing SD card...");
  Serial.print("  CS Pin: GPIO ");
  Serial.println(SD_CS);

  // Initialize SD on default VSPI
  if (SD.begin(SD_CS)) {
    sdAvailable = true;
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.print("  SD Card OK - ");
    Serial.print(cardSize);
    Serial.println(" MB");
  } else {
    sdAvailable = false;
    Serial.println("  SD Card FAILED - continuing without logging");
  }
}

void initDisplay() {
  Serial.println("Initializing display...");

  // NOTE: Tie TFT backlight (BL) directly to 3.3V

  // Initialize ST7789 (320x170)
  tft.init(170, 320);
  tft.setRotation(3);  // Landscape, USB on left
  tft.fillScreen(ST77XX_BLACK);

  // Splash screen
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(3);
  tft.setCursor(60, 50);
  tft.print("AxleWatch");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(95, 90);
  tft.print("Receiver");

  tft.setTextSize(1);
  tft.setCursor(90, 130);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("Homebrew Edition v2.0");

  delay(2000);

  Serial.println("  Display OK (ST7789 320x170)");
}

void initLoRa() {
  Serial.println("Initializing LoRa...");
  Serial.print("  HSPI Bus: SCK=");
  Serial.print(LORA_SCK);
  Serial.print(" MISO=");
  Serial.print(LORA_MISO);
  Serial.print(" MOSI=");
  Serial.println(LORA_MOSI);
  Serial.print("  CS=");
  Serial.print(LORA_CS);
  Serial.print(" RST=");
  Serial.print(LORA_RST);
  Serial.print(" DIO0=");
  Serial.println(LORA_DIO0);

  // Initialize HSPI for LoRa
  hspi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  // Configure LoRa
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  LoRa.setSPI(hspi);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("  ERROR: LoRa init failed!");
    // Flash red LED to indicate error
    while (1) {
      digitalWrite(LED_RED, LOW);
      delay(200);
      digitalWrite(LED_RED, HIGH);
      delay(200);
    }
  }

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.disableCrc();  // Match transmitter config

  Serial.print("  LoRa OK - ");
  Serial.print(LORA_FREQUENCY / 1E6);
  Serial.print(" MHz, SF");
  Serial.print(LORA_SF);
  Serial.print(", BW ");
  Serial.print(LORA_BW / 1000);
  Serial.println(" kHz");
}

void initGPS() {
  Serial.println("Initializing GPS...");
  Serial.print("  RX=GPIO ");
  Serial.print(GPS_RX);
  Serial.print(" TX=GPIO ");
  Serial.println(GPS_TX);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  gpsData.validFix = false;
  gpsData.satellites = 0;

  Serial.println("  GPS OK (9600 baud)");
}

// ======================== UPDATE FUNCTIONS ========================

void updateLoRa() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  // Read packet
  char buffer[128];
  int idx = 0;
  while (LoRa.available() && idx < sizeof(buffer) - 1) {
    buffer[idx++] = (char)LoRa.read();
  }
  buffer[idx] = '\0';

  int rssi = LoRa.packetRssi();

  Serial.print("LoRa RX: ");
  Serial.print(buffer);
  Serial.print(" (RSSI: ");
  Serial.print(rssi);
  Serial.println(" dBm)");

  // Parse packet
  TransmitterData tempTx;
  if (!parseLoRaPacket(buffer, &tempTx)) {
    Serial.println("  Parse failed - invalid packet");
    return;
  }

  tempTx.rssi = rssi;
  tempTx.lastReceived = millis();
  tempTx.active = true;

  // Find or create transmitter slot
  int slot = -1;
  for (int i = 0; i < MAX_TRANSMITTERS; i++) {
    if (strcmp(transmitters[i].txID, tempTx.txID) == 0) {
      slot = i;
      break;
    }
  }

  if (slot == -1) {
    // Find empty slot
    for (int i = 0; i < MAX_TRANSMITTERS; i++) {
      if (!transmitters[i].active && strlen(transmitters[i].txID) == 0) {
        slot = i;
        break;
      }
    }
  }

  if (slot == -1) {
    // Find inactive slot
    for (int i = 0; i < MAX_TRANSMITTERS; i++) {
      if (!transmitters[i].active) {
        slot = i;
        break;
      }
    }
  }

  if (slot >= 0) {
    memcpy(&transmitters[slot], &tempTx, sizeof(TransmitterData));
    Serial.print("  Stored in slot ");
    Serial.println(slot);

    // Log to SD
    logToSD(&transmitters[slot]);
  } else {
    Serial.println("  No available slot!");
  }
}

bool parseLoRaPacket(const char* packet, TransmitterData* tx) {
  char buffer[128];
  strncpy(buffer, packet, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  // Validate characters
  for (int i = 0; buffer[i] != '\0'; i++) {
    char c = buffer[i];
    bool valid = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                 (c >= '0' && c <= '9') || c == '.' || c == ',' ||
                 c == ':' || c == '-';
    if (!valid) return false;
  }

  // Accept TX, TRAILER, or DOLLY prefixes
  bool validPrefix = (strncmp(buffer, "TX", 2) == 0) ||
                     (strncmp(buffer, "TRAILER", 7) == 0) ||
                     (strncmp(buffer, "DOLLY", 5) == 0);
  if (!validPrefix) return false;

  // Find colon
  char* colonPos = strchr(buffer, ':');
  if (!colonPos) return false;

  // Extract TX ID
  int idLen = colonPos - buffer;
  if (idLen >= sizeof(tx->txID)) return false;
  strncpy(tx->txID, buffer, idLen);
  tx->txID[idLen] = '\0';

  // Count commas (should be 9 for 10 values)
  char* dataStart = colonPos + 1;
  int commaCount = 0;
  for (char* p = dataStart; *p; p++) {
    if (*p == ',') commaCount++;
  }
  if (commaCount != 9) return false;

  // Parse temperatures
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
    return false;
  }

  if (count != NUM_TEMP_SENSORS) return false;

  return true;
}

void updateGPS() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (gps.encode(c)) {
      if (gps.location.isValid()) {
        gpsData.latitude = gps.location.lat();
        gpsData.longitude = gps.location.lng();
        gpsData.validFix = true;
      }

      if (gps.speed.isValid()) {
        gpsData.speedKmh = gps.speed.kmph();
      }

      if (gps.satellites.isValid()) {
        gpsData.satellites = gps.satellites.value();
      }
    }
  }
}

void updateAlarms() {
  uint8_t maxLevel = 0;

  for (int i = 0; i < MAX_TRANSMITTERS; i++) {
    if (!transmitters[i].active) continue;

    float ambient = transmitters[i].ambientTemp;

    for (int j = 0; j < NUM_TEMP_SENSORS; j++) {
      float temp = transmitters[i].temps[j];

      // Skip unused sensors (0.0)
      if (temp < 1.0) {
        transmitters[i].alarmLevels[j] = 0;
        continue;
      }

      float delta = temp - ambient;

      if (delta >= DEFAULT_CRIT_OFFSET) {
        transmitters[i].alarmLevels[j] = 2;
        if (maxLevel < 2) maxLevel = 2;
      } else if (delta >= DEFAULT_WARN_OFFSET) {
        transmitters[i].alarmLevels[j] = 1;
        if (maxLevel < 1) maxLevel = 1;
      } else {
        transmitters[i].alarmLevels[j] = 0;
      }
    }
  }

  currentAlarmLevel = maxLevel;
  alarmActive = (maxLevel > 0);

  // Reset mute when alarm clears
  if (!alarmActive) {
    alarmMuted = false;
  }
}

void updateLEDs() {
  // Turn off all LEDs first (active HIGH)
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);

  // Light appropriate LED based on alarm level
  switch (currentAlarmLevel) {
    case 2:  // Critical
      digitalWrite(LED_RED, HIGH);
      break;
    case 1:  // Warning
      digitalWrite(LED_YELLOW, HIGH);
      break;
    default:  // OK
      digitalWrite(LED_GREEN, HIGH);
      break;
  }
}

void updateBuzzer() {
  if (!alarmActive || alarmMuted) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerState = false;
    return;
  }

  // Buzzer pattern based on alarm level
  unsigned long interval = (currentAlarmLevel == 2) ? 300 : 800;

  if (millis() - lastBuzzerToggle >= interval) {
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    lastBuzzerToggle = millis();
  }
}

void handleButton() {
  bool buttonState = digitalRead(BUTTON_PIN);

  // Button pressed (active LOW with pull-up)
  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressStart = millis();
    buttonHeld = false;
  }

  // Button released
  if (buttonState == HIGH && lastButtonState == LOW) {
    unsigned long pressDuration = millis() - buttonPressStart;

    // Debounce
    if (pressDuration < 50) {
      lastButtonState = buttonState;
      return;
    }

    if (pressDuration < 1000) {
      // Short press - cycle display mode or mute alarm
      if (alarmActive && !alarmMuted) {
        // Mute alarm
        alarmMuted = true;
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("Alarm MUTED");
      } else {
        // Cycle display mode
        displayMode = (DisplayMode)((displayMode + 1) % 3);
        Serial.print("Display mode: ");
        Serial.println(displayMode);
      }
      lastDisplayUpdate = 0;  // Force redraw
    } else if (!buttonHeld) {
      // Long press (1+ seconds) - cycle to next transmitter
      cycleTx();
      lastDisplayUpdate = 0;  // Force redraw
    }

    lastButtonPress = millis();
  }

  // Check for held button (long press action while still holding)
  if (buttonState == LOW && (millis() - buttonPressStart > 1000) && !buttonHeld) {
    buttonHeld = true;
    // Long press action already handled on release
  }

  lastButtonState = buttonState;
}

void cycleTx() {
  int startIdx = currentTxIndex;
  do {
    currentTxIndex = (currentTxIndex + 1) % MAX_TRANSMITTERS;
    if (transmitters[currentTxIndex].active) {
      Serial.print("Switched to TX: ");
      Serial.println(transmitters[currentTxIndex].txID);
      return;
    }
  } while (currentTxIndex != startIdx);
}

void updateDisplay() {
  // Rate limit display updates
  if (millis() - lastDisplayUpdate < 1000) return;
  lastDisplayUpdate = millis();

  // Auto-cycle transmitters in overview mode
  if (displayMode == MODE_OVERVIEW && millis() - lastDisplayCycle >= DISPLAY_CYCLE_INTERVAL) {
    cycleTx();
    lastDisplayCycle = millis();
  }

  // Draw appropriate screen
  switch (displayMode) {
    case MODE_OVERVIEW:
      drawOverviewScreen();
      break;
    case MODE_DETAIL:
      drawDetailScreen();
      break;
    case MODE_STATUS:
      drawStatusScreen();
      break;
  }
}

// ======================== DISPLAY FUNCTIONS ========================

uint16_t getAlarmColor(uint8_t level) {
  switch (level) {
    case 2: return ST77XX_RED;
    case 1: return ST77XX_YELLOW;
    default: return ST77XX_GREEN;
  }
}

int countActiveTx() {
  int count = 0;
  for (int i = 0; i < MAX_TRANSMITTERS; i++) {
    if (transmitters[i].active) count++;
  }
  return count;
}

void drawStatusBar() {
  // Top status bar
  tft.fillRect(0, 0, SCREEN_WIDTH, 20, ST77XX_BLACK);

  // GPS status
  tft.setTextSize(1);
  tft.setCursor(5, 6);
  if (gpsData.validFix) {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("GPS:");
    tft.print(gpsData.satellites);
  } else {
    tft.setTextColor(ST77XX_RED);
    tft.print("NO GPS");
  }

  // Speed
  tft.setCursor(70, 6);
  tft.setTextColor(ST77XX_CYAN);
  tft.print(gpsData.speedKmh, 0);
  tft.print("km/h");

  // SD status
  tft.setCursor(130, 6);
  if (sdAvailable) {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("SD OK");
  } else {
    tft.setTextColor(ST77XX_RED);
    tft.print("NO SD");
  }

  // Active TX count
  int txCount = countActiveTx();
  tft.setCursor(180, 6);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("TX:");
  tft.print(txCount);

  // Alarm status
  tft.setCursor(220, 6);
  if (alarmMuted) {
    tft.setTextColor(ST77XX_ORANGE);
    tft.print("MUTED");
  } else if (currentAlarmLevel == 2) {
    tft.setTextColor(ST77XX_RED);
    tft.print("ALARM!");
  } else if (currentAlarmLevel == 1) {
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("WARN");
  } else {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("OK");
  }

  // Mode indicator
  tft.setCursor(280, 6);
  tft.setTextColor(ST77XX_MAGENTA);
  switch (displayMode) {
    case MODE_OVERVIEW: tft.print("[OVR]"); break;
    case MODE_DETAIL:   tft.print("[DTL]"); break;
    case MODE_STATUS:   tft.print("[STS]"); break;
  }

  // Divider line
  tft.drawFastHLine(0, 20, SCREEN_WIDTH, ST77XX_WHITE);
}

void drawOverviewScreen() {
  tft.fillScreen(ST77XX_BLACK);
  drawStatusBar();

  if (countActiveTx() == 0) {
    // No transmitters
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(40, 70);
    tft.print("Waiting for TX...");

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(60, 110);
    tft.print("Listening on 433 MHz");
    return;
  }

  TransmitterData* tx = &transmitters[currentTxIndex];

  // TX ID and RSSI
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(5, 25);
  tft.print(tx->txID);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(200, 30);
  tft.print("RSSI:");
  tft.print(tx->rssi);
  tft.print("dBm");

  // Ambient temperature
  tft.setCursor(5, 45);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Ambient: ");
  tft.print(tx->ambientTemp, 1);
  tft.print("C");

  // Temperature grid (3x3)
  int startY = 60;
  int rowHeight = 36;
  int colWidth = 105;

  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      int idx = row * 3 + col;
      int x = col * colWidth + 5;
      int y = startY + row * rowHeight;

      // Position label
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(x, y);
      tft.print("P");
      tft.print(idx + 1);

      // Temperature value
      float temp = tx->temps[idx];
      uint16_t color = getAlarmColor(tx->alarmLevels[idx]);

      tft.setTextSize(2);
      tft.setTextColor(color);
      tft.setCursor(x, y + 12);

      if (temp < 1.0) {
        tft.print("--.-");
      } else {
        if (temp < 100) tft.print(" ");
        tft.print(temp, 1);
      }
    }
  }

  // Bottom hint
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5, 158);
  tft.print("Press: Mode | Hold: Next TX");
}

void drawDetailScreen() {
  tft.fillScreen(ST77XX_BLACK);
  drawStatusBar();

  if (countActiveTx() == 0) {
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(60, 70);
    tft.print("No Active TX");
    return;
  }

  TransmitterData* tx = &transmitters[currentTxIndex];

  // TX ID header
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(5, 25);
  tft.print(tx->txID);
  tft.setTextSize(1);
  tft.print(" Detail View");

  // List all positions with delta from ambient
  int y = 50;
  for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
    float temp = tx->temps[i];
    float delta = temp - tx->ambientTemp;
    uint16_t color = getAlarmColor(tx->alarmLevels[i]);

    tft.setTextSize(1);
    tft.setCursor(5 + (i % 3) * 105, y + (i / 3) * 35);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("P");
    tft.print(i + 1);
    tft.print(": ");

    tft.setTextColor(color);
    if (temp < 1.0) {
      tft.print("--.-C");
    } else {
      tft.print(temp, 1);
      tft.print("C");

      // Delta
      tft.setTextSize(1);
      tft.setCursor(5 + (i % 3) * 105, y + (i / 3) * 35 + 12);
      tft.setTextColor(ST77XX_WHITE);
      tft.print("(+");
      tft.print(delta, 1);
      tft.print(")");
    }
  }

  // Ambient at bottom
  tft.setCursor(5, 155);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Ambient: ");
  tft.print(tx->ambientTemp, 1);
  tft.print("C | RSSI: ");
  tft.print(tx->rssi);
  tft.print("dBm");
}

void drawStatusScreen() {
  tft.fillScreen(ST77XX_BLACK);
  drawStatusBar();

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(80, 25);
  tft.print("System Status");

  tft.setTextSize(1);
  int y = 50;
  int lineHeight = 18;

  // LoRa
  tft.setCursor(10, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("LoRa: ");
  tft.setTextColor(ST77XX_GREEN);
  tft.print("433MHz SF7 125kHz");
  y += lineHeight;

  // GPS
  tft.setCursor(10, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("GPS: ");
  if (gpsData.validFix) {
    tft.setTextColor(ST77XX_GREEN);
    tft.print(gpsData.latitude, 5);
    tft.print(", ");
    tft.print(gpsData.longitude, 5);
  } else {
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Acquiring... (");
    tft.print(gpsData.satellites);
    tft.print(" sats)");
  }
  y += lineHeight;

  // Speed
  tft.setCursor(10, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Speed: ");
  tft.setTextColor(ST77XX_CYAN);
  tft.print(gpsData.speedKmh, 1);
  tft.print(" km/h");
  y += lineHeight;

  // SD Card
  tft.setCursor(10, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("SD Card: ");
  if (sdAvailable) {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("Logging active");
  } else {
    tft.setTextColor(ST77XX_RED);
    tft.print("Not available");
  }
  y += lineHeight;

  // Active transmitters
  tft.setCursor(10, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Active TX: ");
  tft.setTextColor(ST77XX_CYAN);
  int count = countActiveTx();
  tft.print(count);
  tft.print("/");
  tft.print(MAX_TRANSMITTERS);
  y += lineHeight;

  // List active TX IDs
  if (count > 0) {
    tft.setCursor(10, y);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("IDs: ");
    tft.setTextColor(ST77XX_GREEN);
    for (int i = 0; i < MAX_TRANSMITTERS; i++) {
      if (transmitters[i].active) {
        tft.print(transmitters[i].txID);
        tft.print(" ");
      }
    }
  }
  y += lineHeight;

  // Alarm thresholds
  tft.setCursor(10, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Warn: +");
  tft.print(DEFAULT_WARN_OFFSET, 0);
  tft.print("C  Crit: +");
  tft.print(DEFAULT_CRIT_OFFSET, 0);
  tft.print("C");
}

// ======================== SD LOGGING ========================

void logToSD(TransmitterData* tx) {
  if (!sdAvailable) return;

  File logFile = SD.open("/axlewatch.csv", FILE_APPEND);
  if (!logFile) {
    Serial.println("Failed to open log file");
    return;
  }

  // Write header if file is new
  if (logFile.size() == 0) {
    logFile.print("Timestamp,TxID,");
    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
      logFile.print("P");
      logFile.print(i + 1);
      logFile.print(",");
    }
    logFile.print("Ambient,Lat,Lon,Speed,RSSI");
    logFile.println();
  }

  // Write data
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
  logFile.print(gpsData.latitude, 6);
  logFile.print(",");
  logFile.print(gpsData.longitude, 6);
  logFile.print(",");
  logFile.print(gpsData.speedKmh, 1);
  logFile.print(",");
  logFile.print(tx->rssi);
  logFile.println();

  logFile.close();
}
