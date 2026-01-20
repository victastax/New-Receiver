/*
 * AxleWatch Receiver Firmware v2.0
 * ESP32-WROOM-32 Homebrew Hardware
 *
 * Features:
 * - LoRa 433MHz reception from trailer transmitters
 * - 1.9" ST7789 TFT display (170x320) with simple UI
 * - WiFi AP mode with web dashboard for phone viewing
 * - Configurable alarm thresholds via web interface
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
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <FS.h>
#include <HTTPClient.h>

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
#define BUZZER_PIN      0   // GPIO0 - boot pin but safe with piezo (has internal pull-up)

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

// WiFi AP settings
#define AP_SSID         "AxleWatch-RX-Setup"
#define AP_PASSWORD     "axlewatch123"
#define AP_CHANNEL      6

// ======================== GLOBAL OBJECTS ========================
// SPI Buses
SPIClass hspi(HSPI);  // Dedicated for LoRa

// Display (uses default VSPI)
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// WiFi and Web Server
WebServer server(80);
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
  int satellites;
  bool validFix;
};

struct Config {
  float warnOffset;
  float critOffset;
  char wifiSSID[64];       // Truck's WiFi SSID (e.g., Starlink)
  char wifiPassword[64];   // WiFi password
  char dashboardURL[128];  // Dashboard API endpoint
  char apiKey[64];         // API key for cloud authentication
  bool wifiEnabled;        // Enable WiFi client mode
};

// ======================== GLOBAL VARIABLES ========================

// Display mode enum (must be defined before use)
enum DisplayMode {
  MODE_OVERVIEW,    // Shows current TX temps
  MODE_DETAIL,      // Shows all 9 positions for current TX
  MODE_STATUS       // Shows system status (GPS, SD, etc)
};

TransmitterData transmitters[MAX_TRANSMITTERS];
GPSData gpsData;
Config config;

char deviceID[20] = "";  // Device ID from MAC address (AW-XXXXXXXXXXXX)

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

// Buzzer PWM control
bool buzzerEnabled = false;
bool buzzerOn = false;
unsigned long buzzerInterval = 0;
uint16_t buzzerFreq = 0;
uint8_t buzzerDuty = 0;

// Display refresh control (prevents flickering)
bool forceDisplayRedraw = true;
DisplayMode lastDisplayMode = MODE_OVERVIEW;
int lastTxIndex = -1;
uint8_t lastAlarmLevel = 255;

// Button state
bool lastButtonState = HIGH;
unsigned long lastButtonPress = 0;
unsigned long buttonPressStart = 0;
bool buttonHeld = false;

// SD card
bool sdAvailable = false;

// WiFi state
bool apModeActive = true;  // AP mode for local config
IPAddress apIP(192, 168, 4, 1);

// WiFi Client state (for connecting to truck's WiFi)
bool wifiClientConnected = false;
unsigned long lastDashboardUpdate = 0;
unsigned long lastWifiReconnectAttempt = 0;
#define DASHBOARD_UPDATE_INTERVAL 60000  // Send data every 60 seconds
#define WIFI_RECONNECT_INTERVAL   30000  // Retry connection every 30 seconds

// Display state
DisplayMode displayMode = MODE_OVERVIEW;

// ======================== FUNCTION PROTOTYPES ========================
void initLoRa();
void initGPS();
void initDisplay();
void initSD();
void initLEDs();
void initBuzzer();
void initButton();
void initWiFi();
void initWebServer();
void loadConfig();
void saveConfig();

void updateLoRa();
void updateGPS();
void updateDisplay();
void updateAlarms();
void updateLEDs();
void updateBuzzer();
void buzzerSilence();
void buzzerOff();
void buzzerWarning();
void buzzerAlarm();
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

// Web handlers
void handleRoot();
void handleLive();
void handleConfig();
void handleSaveConfig();
void handleWifiConfig();
void handleSaveWifi();
void handleApiData();

// WiFi/Dashboard
void connectToWiFi();
void sendToDashboard();
String getISOTimestamp();
void getAlertInfo(TransmitterData* tx, String& level, String& message);

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

  // Load saved configuration
  loadConfig();

  // Initialize all subsystems
  initLEDs();
  initButton();
  initBuzzer();

  // IMPORTANT: Initialize SD before display (shared SPI bus)
  initSD();
  initDisplay();

  initLoRa();
  initGPS();
  initWiFi();
  initWebServer();

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
  // Handle web server
  server.handleClient();

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

  // Maintain WiFi client connection
  connectToWiFi();

  // Send data to dashboard
  sendToDashboard();
}

// ======================== CONFIG FUNCTIONS ========================

void loadConfig() {
  prefs.begin("axlewatch", false);
  config.warnOffset = prefs.getFloat("warnOffset", DEFAULT_WARN_OFFSET);
  config.critOffset = prefs.getFloat("critOffset", DEFAULT_CRIT_OFFSET);
  config.wifiEnabled = prefs.getBool("wifiEnabled", false);

  // Load WiFi credentials and API settings
  String ssid = prefs.getString("wifiSSID", "");
  String pass = prefs.getString("wifiPass", "");
  String url = prefs.getString("dashURL", "https://axlewatch.com/api/telemetry");
  String apiKey = prefs.getString("apiKey", "");

  strncpy(config.wifiSSID, ssid.c_str(), sizeof(config.wifiSSID) - 1);
  strncpy(config.wifiPassword, pass.c_str(), sizeof(config.wifiPassword) - 1);
  strncpy(config.dashboardURL, url.c_str(), sizeof(config.dashboardURL) - 1);
  strncpy(config.apiKey, apiKey.c_str(), sizeof(config.apiKey) - 1);

  prefs.end();

  Serial.print("Config loaded - Warn: +");
  Serial.print(config.warnOffset);
  Serial.print("C, Crit: +");
  Serial.print(config.critOffset);
  Serial.println("C");

  if (config.wifiEnabled && strlen(config.wifiSSID) > 0) {
    Serial.print("WiFi Client: ");
    Serial.println(config.wifiSSID);
  }
}

void saveConfig() {
  prefs.begin("axlewatch", false);
  prefs.putFloat("warnOffset", config.warnOffset);
  prefs.putFloat("critOffset", config.critOffset);
  prefs.putBool("wifiEnabled", config.wifiEnabled);
  prefs.putString("wifiSSID", config.wifiSSID);
  prefs.putString("wifiPass", config.wifiPassword);
  prefs.putString("dashURL", config.dashboardURL);
  prefs.putString("apiKey", config.apiKey);
  prefs.end();

  Serial.println("Config saved");
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

  // Set pin low initially
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Quick beep test using PWM
  ledcAttach(BUZZER_PIN, 2000, 8);
  ledcWrite(BUZZER_PIN, 128);
  delay(100);

  // Detach PWM and silence completely
  ledcDetach(BUZZER_PIN);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("  Buzzer OK (PWM)");
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

  // Get MAC address for device ID (from chip's efuse - always available)
  uint64_t chipid = ESP.getEfuseMac();
  snprintf(deviceID, sizeof(deviceID), "AW-%02X%02X%02X%02X%02X%02X",
           (uint8_t)(chipid), (uint8_t)(chipid >> 8), (uint8_t)(chipid >> 16),
           (uint8_t)(chipid >> 24), (uint8_t)(chipid >> 32), (uint8_t)(chipid >> 40));

  Serial.println("========================================");
  Serial.print("  DEVICE ID: ");
  Serial.println(deviceID);
  Serial.println("  (Use this to register on AxleWatch.com)");
  Serial.println("========================================");

  // NOTE: Tie TFT backlight (BL) directly to 3.3V

  // Initialize ST7789 (320x170)
  tft.init(170, 320);
  tft.setRotation(3);  // Landscape, USB on left
  tft.fillScreen(ST77XX_BLACK);

  // Splash screen
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(3);
  tft.setCursor(60, 20);
  tft.print("AxleWatch");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(95, 55);
  tft.print("Receiver");

  tft.setTextSize(1);
  tft.setCursor(70, 85);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("Homebrew Edition v2.0");

  // Display Device ID
  tft.setCursor(10, 110);
  tft.setTextColor(ST77XX_GREEN);
  tft.print("ID: ");
  tft.setTextColor(ST77XX_WHITE);
  tft.print(deviceID);

  // WiFi info
  tft.setCursor(10, 130);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("WiFi: ");
  tft.setTextColor(ST77XX_WHITE);
  tft.print(AP_SSID);

  tft.setCursor(10, 150);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Dashboard: ");
  tft.setTextColor(ST77XX_WHITE);
  tft.print("http://192.168.4.1");

  delay(5000);  // Show splash longer so user can note info

  Serial.println("  Display OK (ST7789 320x170)");
}

void initWiFi() {
  Serial.println("Initializing WiFi...");

  // Use AP+STA mode if client WiFi is configured, otherwise just AP
  if (config.wifiEnabled && strlen(config.wifiSSID) > 0) {
    WiFi.mode(WIFI_AP_STA);
    Serial.println("  Mode: AP + Station");
  } else {
    WiFi.mode(WIFI_AP);
    Serial.println("  Mode: AP only");
  }

  // Start Access Point (always available for local config)
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);

  Serial.print("  AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("  AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Connect to truck's WiFi if configured
  if (config.wifiEnabled && strlen(config.wifiSSID) > 0) {
    Serial.print("  Connecting to: ");
    Serial.println(config.wifiSSID);

    WiFi.begin(config.wifiSSID, config.wifiPassword);

    // Wait up to 10 seconds for connection
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      wifiClientConnected = true;
      Serial.println();
      Serial.print("  Connected! IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println();
      Serial.println("  Failed to connect - will retry later");
    }
  }

  Serial.println("  WiFi OK");
}

void connectToWiFi() {
  // Try to reconnect to truck's WiFi
  if (!config.wifiEnabled || strlen(config.wifiSSID) == 0) {
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiClientConnected = true;
    return;
  }

  unsigned long now = millis();
  if (now - lastWifiReconnectAttempt < WIFI_RECONNECT_INTERVAL) {
    return;
  }
  lastWifiReconnectAttempt = now;

  Serial.print("Reconnecting to WiFi: ");
  Serial.println(config.wifiSSID);

  WiFi.begin(config.wifiSSID, config.wifiPassword);

  // Brief wait
  delay(1000);

  if (WiFi.status() == WL_CONNECTED) {
    wifiClientConnected = true;
    Serial.println("WiFi reconnected!");
  } else {
    wifiClientConnected = false;
  }
}

void initWebServer() {
  Serial.println("Initializing web server...");

  server.on("/", handleRoot);
  server.on("/live", handleLive);
  server.on("/config", handleConfig);
  server.on("/save", HTTP_POST, handleSaveConfig);
  server.on("/wifi", handleWifiConfig);
  server.on("/savewifi", HTTP_POST, handleSaveWifi);
  server.on("/api/data", handleApiData);

  server.begin();
  Serial.println("  Web server OK");
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
      digitalWrite(LED_RED, HIGH);
      delay(200);
      digitalWrite(LED_RED, LOW);
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

// ======================== WEB HANDLERS ========================

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>AxleWatch Receiver</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { box-sizing: border-box; }
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
           margin: 0; padding: 20px; background: #1a1a2e; color: #eee; }
    .container { max-width: 600px; margin: 0 auto; }
    h1 { color: #00d4ff; text-align: center; margin-bottom: 10px; }
    .subtitle { text-align: center; color: #888; margin-bottom: 20px; }
    .card { background: #16213e; border-radius: 12px; padding: 20px; margin-bottom: 15px; }
    .card h2 { margin-top: 0; color: #00d4ff; font-size: 18px; }
    .btn { display: block; width: 100%; padding: 15px; margin: 10px 0;
           border: none; border-radius: 8px; font-size: 16px; cursor: pointer;
           text-decoration: none; text-align: center; }
    .btn-primary { background: #00d4ff; color: #000; }
    .btn-secondary { background: #0f3460; color: #fff; }
    .info { display: flex; justify-content: space-between; padding: 8px 0;
            border-bottom: 1px solid #0f3460; }
    .info:last-child { border-bottom: none; }
    .label { color: #888; }
    .value { color: #00d4ff; font-weight: bold; }
  </style>
</head>
<body>
  <div class="container">
    <h1>AxleWatch</h1>
    <p class="subtitle">Receiver Dashboard</p>

    <div class="card">
      <h2>Device Info</h2>
      <div class="info"><span class="label">Device ID</span><span class="value">)rawliteral" + String(deviceID) + R"rawliteral(</span></div>
      <div class="info"><span class="label">AP SSID</span><span class="value">)rawliteral" + String(AP_SSID) + R"rawliteral(</span></div>
      <div class="info"><span class="label">AP IP</span><span class="value">192.168.4.1</span></div>
      <div class="info"><span class="label">WiFi Client</span><span class="value" style="color:)rawliteral" + String(wifiClientConnected ? "#00ff88" : "#ff4444") + R"rawliteral(">)rawliteral" + String(wifiClientConnected ? "Connected" : "Not Connected") + R"rawliteral(</span></div>
    </div>

    <a href="/live" class="btn btn-primary">Live Dashboard</a>
    <a href="/config" class="btn btn-secondary">Alarm Settings</a>
    <a href="/wifi" class="btn btn-secondary">WiFi Settings</a>
  </div>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

void handleLive() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>AxleWatch Live</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { box-sizing: border-box; }
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
           margin: 0; padding: 10px; background: #1a1a2e; color: #eee; }
    h1 { color: #00d4ff; text-align: center; font-size: 20px; margin: 10px 0; }
    .status-bar { display: flex; justify-content: space-around; padding: 10px;
                  background: #16213e; border-radius: 8px; margin-bottom: 10px; font-size: 12px; }
    .status-item { text-align: center; }
    .status-label { color: #888; }
    .status-value { font-weight: bold; }
    .ok { color: #00ff88; }
    .warn { color: #ffaa00; }
    .crit { color: #ff4444; }
    .tx-card { background: #16213e; border-radius: 12px; padding: 15px; margin-bottom: 10px; }
    .tx-header { display: flex; justify-content: space-between; margin-bottom: 10px; }
    .tx-id { font-size: 18px; font-weight: bold; color: #00d4ff; }
    .tx-rssi { color: #888; font-size: 12px; }
    .temp-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
    .temp-cell { background: #0f3460; border-radius: 8px; padding: 10px; text-align: center; }
    .temp-label { font-size: 11px; color: #888; }
    .temp-value { font-size: 20px; font-weight: bold; }
    .ambient { margin-top: 10px; text-align: center; color: #888; }
    .no-tx { text-align: center; padding: 40px; color: #888; }
    .refresh-info { text-align: center; color: #666; font-size: 11px; margin-top: 10px; }
    a { color: #00d4ff; }
  </style>
</head>
<body>
  <h1>AxleWatch Live</h1>

  <div class="status-bar">
    <div class="status-item">
      <div class="status-label">GPS</div>
      <div class="status-value" id="gps">--</div>
    </div>
    <div class="status-item">
      <div class="status-label">Speed</div>
      <div class="status-value" id="speed">-- km/h</div>
    </div>
    <div class="status-item">
      <div class="status-label">Active TX</div>
      <div class="status-value" id="txcount">0</div>
    </div>
    <div class="status-item">
      <div class="status-label">Status</div>
      <div class="status-value" id="alarm">--</div>
    </div>
  </div>

  <div id="transmitters"></div>

  <p class="refresh-info">Auto-refresh every 2 seconds | <a href="/">Home</a></p>

  <script>
    function getAlarmClass(level) {
      if (level == 2) return 'crit';
      if (level == 1) return 'warn';
      return 'ok';
    }

    function updateData() {
      fetch('/api/data')
        .then(r => r.json())
        .then(data => {
          // Update status bar
          document.getElementById('gps').innerHTML = data.gps.valid ?
            '<span class="ok">' + data.gps.sats + ' sats</span>' :
            '<span class="warn">No Fix</span>';
          document.getElementById('speed').textContent = data.gps.speed.toFixed(0) + ' km/h';
          document.getElementById('txcount').textContent = data.activeTx;

          let alarmEl = document.getElementById('alarm');
          if (data.alarmLevel == 2) {
            alarmEl.innerHTML = '<span class="crit">ALARM!</span>';
          } else if (data.alarmLevel == 1) {
            alarmEl.innerHTML = '<span class="warn">WARNING</span>';
          } else {
            alarmEl.innerHTML = '<span class="ok">OK</span>';
          }

          // Update transmitters
          let txHtml = '';
          if (data.transmitters.length == 0) {
            txHtml = '<div class="no-tx">Waiting for transmitters...</div>';
          } else {
            data.transmitters.forEach(tx => {
              txHtml += '<div class="tx-card">';
              txHtml += '<div class="tx-header">';
              txHtml += '<span class="tx-id">' + tx.id + '</span>';
              txHtml += '<span class="tx-rssi">RSSI: ' + tx.rssi + ' dBm</span>';
              txHtml += '</div>';
              txHtml += '<div class="temp-grid">';
              for (let i = 0; i < 9; i++) {
                let temp = tx.temps[i];
                let alarm = tx.alarms[i];
                let tempStr = temp < 1 ? '--' : temp.toFixed(1);
                txHtml += '<div class="temp-cell">';
                txHtml += '<div class="temp-label">P' + (i+1) + '</div>';
                txHtml += '<div class="temp-value ' + getAlarmClass(alarm) + '">' + tempStr + '</div>';
                txHtml += '</div>';
              }
              txHtml += '</div>';
              txHtml += '<div class="ambient">Ambient: ' + tx.ambient.toFixed(1) + '°C</div>';
              txHtml += '</div>';
            });
          }
          document.getElementById('transmitters').innerHTML = txHtml;
        })
        .catch(err => console.error('Error:', err));
    }

    updateData();
    setInterval(updateData, 2000);
  </script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

void handleConfig() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>AxleWatch Settings</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { box-sizing: border-box; }
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
           margin: 0; padding: 20px; background: #1a1a2e; color: #eee; }
    .container { max-width: 400px; margin: 0 auto; }
    h1 { color: #00d4ff; text-align: center; }
    .card { background: #16213e; border-radius: 12px; padding: 20px; margin-bottom: 15px; }
    label { display: block; color: #888; margin-bottom: 5px; font-size: 14px; }
    input { width: 100%; padding: 12px; border: 1px solid #0f3460; border-radius: 8px;
            background: #0f3460; color: #fff; font-size: 16px; margin-bottom: 15px; }
    .btn { display: block; width: 100%; padding: 15px; margin: 10px 0;
           border: none; border-radius: 8px; font-size: 16px; cursor: pointer; }
    .btn-primary { background: #00d4ff; color: #000; }
    .btn-secondary { background: #0f3460; color: #fff; text-decoration: none; text-align: center; }
    .hint { font-size: 12px; color: #666; margin-top: -10px; margin-bottom: 15px; }
    .success { background: #00ff88; color: #000; padding: 10px; border-radius: 8px;
               text-align: center; margin-bottom: 15px; display: none; }
  </style>
</head>
<body>
  <div class="container">
    <h1>Settings</h1>

    <div class="success" id="success">Settings saved!</div>

    <form id="configForm">
      <div class="card">
        <h3 style="margin-top:0; color:#00d4ff;">Alarm Thresholds</h3>
        <label>Warning Threshold (°C above ambient)</label>
        <input type="number" id="warnOffset" value=")rawliteral" + String(config.warnOffset, 0) + R"rawliteral(" step="1" min="10" max="100">
        <p class="hint">Yellow LED and slow beep when exceeded</p>

        <label>Critical Threshold (°C above ambient)</label>
        <input type="number" id="critOffset" value=")rawliteral" + String(config.critOffset, 0) + R"rawliteral(" step="1" min="20" max="150">
        <p class="hint">Red LED and fast beep when exceeded</p>
      </div>

      <button type="submit" class="btn btn-primary">Save Settings</button>
    </form>

    <a href="/" class="btn btn-secondary">Back to Home</a>
  </div>

  <script>
    document.getElementById('configForm').addEventListener('submit', function(e) {
      e.preventDefault();

      let formData = new FormData();
      formData.append('warnOffset', document.getElementById('warnOffset').value);
      formData.append('critOffset', document.getElementById('critOffset').value);

      fetch('/save', {
        method: 'POST',
        body: formData
      })
      .then(r => r.text())
      .then(data => {
        document.getElementById('success').style.display = 'block';
        setTimeout(() => {
          document.getElementById('success').style.display = 'none';
        }, 3000);
      })
      .catch(err => alert('Error saving settings'));
    });
  </script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

void handleSaveConfig() {
  if (server.hasArg("warnOffset")) {
    config.warnOffset = server.arg("warnOffset").toFloat();
  }
  if (server.hasArg("critOffset")) {
    config.critOffset = server.arg("critOffset").toFloat();
  }

  // Validate
  if (config.warnOffset < 10) config.warnOffset = 10;
  if (config.warnOffset > 100) config.warnOffset = 100;
  if (config.critOffset < 20) config.critOffset = 20;
  if (config.critOffset > 150) config.critOffset = 150;
  if (config.critOffset <= config.warnOffset) {
    config.critOffset = config.warnOffset + 10;
  }

  saveConfig();

  server.send(200, "text/plain", "OK");
}

void handleWifiConfig() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>AxleWatch WiFi Settings</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { box-sizing: border-box; }
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
           margin: 0; padding: 20px; background: #1a1a2e; color: #eee; }
    .container { max-width: 400px; margin: 0 auto; }
    h1 { color: #00d4ff; text-align: center; }
    .card { background: #16213e; border-radius: 12px; padding: 20px; margin-bottom: 15px; }
    label { display: block; color: #888; margin-bottom: 5px; font-size: 14px; }
    input[type="text"], input[type="password"], input[type="url"] {
      width: 100%; padding: 12px; border: 1px solid #0f3460; border-radius: 8px;
      background: #0f3460; color: #fff; font-size: 16px; margin-bottom: 15px; }
    .toggle { display: flex; align-items: center; gap: 10px; margin-bottom: 15px; }
    .toggle input { width: auto; margin: 0; }
    .btn { display: block; width: 100%; padding: 15px; margin: 10px 0;
           border: none; border-radius: 8px; font-size: 16px; cursor: pointer; }
    .btn-primary { background: #00d4ff; color: #000; }
    .btn-secondary { background: #0f3460; color: #fff; text-decoration: none; text-align: center; }
    .hint { font-size: 12px; color: #666; margin-top: -10px; margin-bottom: 15px; }
    .success { background: #00ff88; color: #000; padding: 10px; border-radius: 8px;
               text-align: center; margin-bottom: 15px; display: none; }
    .status { padding: 10px; border-radius: 8px; margin-bottom: 15px; text-align: center; }
    .connected { background: #00ff88; color: #000; }
    .disconnected { background: #ff4444; color: #fff; }
  </style>
</head>
<body>
  <div class="container">
    <h1>WiFi Settings</h1>

    <div class="success" id="success">Settings saved! Device will reconnect...</div>

    <div class="status )rawliteral" + String(wifiClientConnected ? "connected" : "disconnected") + R"rawliteral(">
      )rawliteral" + String(wifiClientConnected ? "Connected to WiFi" : "Not connected to WiFi") + R"rawliteral(
    </div>

    <form id="wifiForm">
      <div class="card">
        <h3 style="margin-top:0; color:#00d4ff;">Truck WiFi (Starlink)</h3>

        <div class="toggle">
          <input type="checkbox" id="wifiEnabled" )rawliteral" + String(config.wifiEnabled ? "checked" : "") + R"rawliteral(>
          <label style="margin:0; color:#fff;">Enable WiFi Connection</label>
        </div>

        <label>WiFi Network Name (SSID)</label>
        <input type="text" id="wifiSSID" value=")rawliteral" + String(config.wifiSSID) + R"rawliteral(" placeholder="Enter WiFi name">

        <label>WiFi Password</label>
        <input type="password" id="wifiPassword" value=")rawliteral" + String(config.wifiPassword) + R"rawliteral(" placeholder="Enter password">
      </div>

      <div class="card">
        <h3 style="margin-top:0; color:#00d4ff;">Cloud Dashboard</h3>

        <label>Dashboard URL</label>
        <input type="url" id="dashboardURL" value=")rawliteral" + String(config.dashboardURL) + R"rawliteral(" placeholder="https://axlewatch.com/api/telemetry">
        <p class="hint">The API endpoint where temperature data will be sent</p>

        <label>API Key</label>
        <input type="text" id="apiKey" value=")rawliteral" + String(config.apiKey) + R"rawliteral(" placeholder="Enter your API key">
        <p class="hint">Your AxleWatch API key for authentication</p>
      </div>

      <button type="submit" class="btn btn-primary">Save WiFi Settings</button>
    </form>

    <a href="/" class="btn btn-secondary">Back to Home</a>
  </div>

  <script>
    document.getElementById('wifiForm').addEventListener('submit', function(e) {
      e.preventDefault();

      let formData = new FormData();
      formData.append('wifiEnabled', document.getElementById('wifiEnabled').checked ? '1' : '0');
      formData.append('wifiSSID', document.getElementById('wifiSSID').value);
      formData.append('wifiPassword', document.getElementById('wifiPassword').value);
      formData.append('dashboardURL', document.getElementById('dashboardURL').value);
      formData.append('apiKey', document.getElementById('apiKey').value);

      fetch('/savewifi', {
        method: 'POST',
        body: formData
      })
      .then(r => r.text())
      .then(data => {
        document.getElementById('success').style.display = 'block';
        setTimeout(() => {
          window.location.reload();
        }, 3000);
      })
      .catch(err => alert('Error saving settings'));
    });
  </script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

void handleSaveWifi() {
  if (server.hasArg("wifiEnabled")) {
    config.wifiEnabled = (server.arg("wifiEnabled") == "1");
  }
  if (server.hasArg("wifiSSID")) {
    strncpy(config.wifiSSID, server.arg("wifiSSID").c_str(), sizeof(config.wifiSSID) - 1);
  }
  if (server.hasArg("wifiPassword")) {
    strncpy(config.wifiPassword, server.arg("wifiPassword").c_str(), sizeof(config.wifiPassword) - 1);
  }
  if (server.hasArg("dashboardURL")) {
    strncpy(config.dashboardURL, server.arg("dashboardURL").c_str(), sizeof(config.dashboardURL) - 1);
  }
  if (server.hasArg("apiKey")) {
    strncpy(config.apiKey, server.arg("apiKey").c_str(), sizeof(config.apiKey) - 1);
  }

  saveConfig();

  // Attempt to connect with new credentials
  if (config.wifiEnabled && strlen(config.wifiSSID) > 0) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(config.wifiSSID, config.wifiPassword);
  }

  server.send(200, "text/plain", "OK");
}

void handleApiData() {
  String json = "{";

  // GPS data
  json += "\"gps\":{";
  json += "\"valid\":" + String(gpsData.validFix ? "true" : "false") + ",";
  json += "\"lat\":" + String(gpsData.latitude, 6) + ",";
  json += "\"lon\":" + String(gpsData.longitude, 6) + ",";
  json += "\"speed\":" + String(gpsData.speedKmh, 1) + ",";
  json += "\"sats\":" + String(gpsData.satellites);
  json += "},";

  // Alarm level
  json += "\"alarmLevel\":" + String(currentAlarmLevel) + ",";
  json += "\"alarmMuted\":" + String(alarmMuted ? "true" : "false") + ",";

  // Active transmitter count
  int txCount = countActiveTx();
  json += "\"activeTx\":" + String(txCount) + ",";

  // Transmitters array
  json += "\"transmitters\":[";
  bool first = true;
  for (int i = 0; i < MAX_TRANSMITTERS; i++) {
    if (transmitters[i].active) {
      if (!first) json += ",";
      first = false;

      json += "{";
      json += "\"id\":\"" + String(transmitters[i].txID) + "\",";
      json += "\"rssi\":" + String(transmitters[i].rssi) + ",";
      json += "\"ambient\":" + String(transmitters[i].ambientTemp, 1) + ",";

      json += "\"temps\":[";
      for (int j = 0; j < NUM_TEMP_SENSORS; j++) {
        if (j > 0) json += ",";
        json += String(transmitters[i].temps[j], 1);
      }
      json += "],";

      json += "\"alarms\":[";
      for (int j = 0; j < NUM_TEMP_SENSORS; j++) {
        if (j > 0) json += ",";
        json += String(transmitters[i].alarmLevels[j]);
      }
      json += "]";

      json += "}";
    }
  }
  json += "]";

  // Config
  json += ",\"config\":{";
  json += "\"warnOffset\":" + String(config.warnOffset, 1) + ",";
  json += "\"critOffset\":" + String(config.critOffset, 1);
  json += "}";

  json += "}";

  server.send(200, "application/json", json);
}

// ======================== DASHBOARD FUNCTIONS ========================

// Get ISO timestamp string
String getISOTimestamp() {
  // Use GPS time if available, otherwise use millis-based approximation
  char timestamp[32];
  if (gpsData.validFix && gps.date.isValid() && gps.time.isValid()) {
    snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    // Fallback: Unix epoch + millis (not accurate but provides a timestamp)
    unsigned long secs = millis() / 1000;
    snprintf(timestamp, sizeof(timestamp), "1970-01-01T%02lu:%02lu:%02luZ",
             (secs / 3600) % 24, (secs / 60) % 60, secs % 60);
  }
  return String(timestamp);
}

// Find highest alarm level and message for a transmitter
void getAlertInfo(TransmitterData* tx, String& level, String& message) {
  float maxTemp = 0;
  int maxAlarm = 0;
  int maxHub = 0;

  for (int j = 0; j < NUM_TEMP_SENSORS; j++) {
    if (tx->alarmLevels[j] > maxAlarm) {
      maxAlarm = tx->alarmLevels[j];
      maxTemp = tx->temps[j];
      maxHub = j + 1;
    } else if (tx->alarmLevels[j] == maxAlarm && tx->temps[j] > maxTemp) {
      maxTemp = tx->temps[j];
      maxHub = j + 1;
    }
  }

  if (maxAlarm == 2) {
    level = "critical";
    message = "Critical temperature on hub " + String(maxHub) + ": " + String(maxTemp, 1) + "°C";
  } else if (maxAlarm == 1) {
    level = "warning";
    message = "High temperature on hub " + String(maxHub) + ": " + String(maxTemp, 1) + "°C";
  } else {
    level = "";
    message = "";
  }
}

void sendToDashboard() {
  // Only send if connected and configured
  if (!wifiClientConnected || !config.wifiEnabled) {
    return;
  }

  if (strlen(config.dashboardURL) == 0 || strlen(config.apiKey) == 0) {
    return;
  }

  // Rate limit
  unsigned long now = millis();
  if (now - lastDashboardUpdate < DASHBOARD_UPDATE_INTERVAL) {
    return;
  }
  lastDashboardUpdate = now;

  // Check WiFi status
  if (WiFi.status() != WL_CONNECTED) {
    wifiClientConnected = false;
    return;
  }

  // Send data for each active transmitter
  for (int i = 0; i < MAX_TRANSMITTERS; i++) {
    if (!transmitters[i].active) continue;

    TransmitterData* tx = &transmitters[i];

    // Build JSON payload per AxleWatch API spec
    String json = "{";

    // Device ID (receiver MAC)
    json += "\"device_id\":\"" + String(deviceID) + "\",";

    // Timestamp in ISO format
    json += "\"timestamp\":\"" + getISOTimestamp() + "\",";

    // Trailer ID (transmitter ID)
    json += "\"trailer_id\":\"" + String(tx->txID) + "\",";

    // Readings object with hub_1 through hub_8 and ambient_temp
    json += "\"readings\":{";
    for (int j = 0; j < 8; j++) {
      json += "\"hub_" + String(j + 1) + "\":" + String(tx->temps[j], 1);
      json += ",";
    }
    json += "\"ambient_temp\":" + String(tx->ambientTemp, 1);
    json += "},";

    // Location object
    json += "\"location\":{";
    json += "\"latitude\":" + String(gpsData.latitude, 6) + ",";
    json += "\"longitude\":" + String(gpsData.longitude, 6) + ",";
    json += "\"speed\":" + String(gpsData.speedKmh, 1);
    json += "}";

    // Alert object (optional, only if there's a warning or critical)
    String alertLevel, alertMessage;
    getAlertInfo(tx, alertLevel, alertMessage);
    if (alertLevel.length() > 0) {
      json += ",\"alert\":{";
      json += "\"level\":\"" + alertLevel + "\",";
      json += "\"message\":\"" + alertMessage + "\"";
      json += "}";
    }

    json += "}";

    // Send HTTP POST
    HTTPClient http;
    http.begin(config.dashboardURL);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer " + String(config.apiKey));

    Serial.print("[CloudUpload] Posting to: ");
    Serial.println(config.dashboardURL);
    Serial.print("[CloudUpload] Payload size: ");
    Serial.print(json.length());
    Serial.println(" bytes");

    int httpCode = http.POST(json);

    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK) {
        Serial.print("[CloudUpload] Success for trailer ");
        Serial.println(tx->txID);
      } else {
        Serial.print("[CloudUpload] HTTP error ");
        Serial.print(httpCode);
        Serial.print(" for trailer ");
        Serial.println(tx->txID);
      }
    } else {
      Serial.print("[CloudUpload] Connection failed: ");
      Serial.println(http.errorToString(httpCode));
    }

    http.end();

    // Small delay between uploads if multiple trailers
    delay(100);
  }
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

    // Trigger display update
    forceDisplayRedraw = true;
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

      if (delta >= config.critOffset) {
        transmitters[i].alarmLevels[j] = 2;
        if (maxLevel < 2) maxLevel = 2;
      } else if (delta >= config.warnOffset) {
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

// Silence the buzzer pin completely
void buzzerSilence() {
  ledcDetach(BUZZER_PIN);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void buzzerOff() {
  if (buzzerEnabled || buzzerOn) {
    buzzerSilence();
  }
  buzzerEnabled = false;
  buzzerOn = false;
}

void buzzerWarning() {
  buzzerEnabled = true;
  buzzerInterval = 1000;  // Slow beep
  buzzerFreq = 1800;      // Lower pitch
  buzzerDuty = 50;
}

void buzzerAlarm() {
  buzzerEnabled = true;
  buzzerInterval = 300;   // Fast beep
  buzzerFreq = 3000;      // Higher pitch
  buzzerDuty = 180;
}

void updateBuzzer() {
  // Handle alarm state changes
  if (!alarmActive || alarmMuted) {
    if (buzzerEnabled || buzzerOn) {
      buzzerOff();
    }
    return;
  }

  // Set buzzer mode based on alarm level
  if (currentAlarmLevel == 2 && buzzerFreq != 3000) {
    buzzerAlarm();
  } else if (currentAlarmLevel == 1 && buzzerFreq != 1800) {
    buzzerWarning();
  }

  if (!buzzerEnabled) return;

  // Toggle buzzer on/off for beeping pattern
  unsigned long now = millis();
  if (now - lastBuzzerToggle >= buzzerInterval) {
    lastBuzzerToggle = now;
    buzzerOn = !buzzerOn;
    if (buzzerOn) {
      // Attach PWM, play tone
      ledcAttach(BUZZER_PIN, buzzerFreq, 8);
      ledcWrite(BUZZER_PIN, buzzerDuty);
    } else {
      // Completely detach PWM and ground the pin
      buzzerSilence();
    }
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
        buzzerOff();
        Serial.println("Alarm MUTED");
      } else {
        // Cycle display mode
        displayMode = (DisplayMode)((displayMode + 1) % 3);
        Serial.print("Display mode: ");
        Serial.println(displayMode);
      }
      forceDisplayRedraw = true;  // Force redraw
    } else if (!buttonHeld) {
      // Long press (1+ seconds) - cycle to next transmitter
      cycleTx();
      forceDisplayRedraw = true;  // Force redraw
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
  // Rate limit display updates (check every 500ms, but only redraw when needed)
  if (millis() - lastDisplayUpdate < 500) return;
  lastDisplayUpdate = millis();

  // Auto-cycle transmitters in overview mode
  if (displayMode == MODE_OVERVIEW && millis() - lastDisplayCycle >= DISPLAY_CYCLE_INTERVAL) {
    int oldIdx = currentTxIndex;
    cycleTx();
    if (currentTxIndex != oldIdx) {
      forceDisplayRedraw = true;
    }
    lastDisplayCycle = millis();
  }

  // Check if we need to redraw (mode changed, TX changed, or alarm level changed)
  bool needsRedraw = forceDisplayRedraw ||
                     (displayMode != lastDisplayMode) ||
                     (currentTxIndex != lastTxIndex) ||
                     (currentAlarmLevel != lastAlarmLevel);

  if (!needsRedraw) return;

  // Update tracking variables
  lastDisplayMode = displayMode;
  lastTxIndex = currentTxIndex;
  lastAlarmLevel = currentAlarmLevel;
  forceDisplayRedraw = false;

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
  tft.setCursor(60, 6);
  tft.setTextColor(ST77XX_CYAN);
  tft.print(gpsData.speedKmh, 0);
  tft.print("km/h");

  // WiFi indicator
  tft.setCursor(115, 6);
  tft.setTextColor(ST77XX_GREEN);
  tft.print("WiFi");

  // Active TX count
  int txCount = countActiveTx();
  tft.setCursor(155, 6);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("TX:");
  tft.print(txCount);

  // Alarm status
  tft.setCursor(195, 6);
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
  tft.setCursor(260, 6);
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
    tft.setCursor(40, 55);
    tft.print("Waiting for TX...");

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(50, 90);
    tft.print("Listening on 433 MHz");

    tft.setCursor(35, 110);
    tft.setTextColor(ST77XX_CYAN);
    tft.print("Phone: http://192.168.4.1");
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
  int y = 48;
  int lineHeight = 16;

  // WiFi
  tft.setCursor(10, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("WiFi: ");
  tft.setTextColor(ST77XX_GREEN);
  tft.print(AP_SSID);
  y += lineHeight;

  // IP
  tft.setCursor(10, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Dashboard: ");
  tft.setTextColor(ST77XX_CYAN);
  tft.print("http://192.168.4.1");
  y += lineHeight;

  // GPS
  tft.setCursor(10, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("GPS: ");
  if (gpsData.validFix) {
    tft.setTextColor(ST77XX_GREEN);
    tft.print(gpsData.satellites);
    tft.print(" sats, ");
    tft.print(gpsData.speedKmh, 0);
    tft.print(" km/h");
  } else {
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Acquiring...");
  }
  y += lineHeight;

  // Thresholds
  tft.setCursor(10, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Warn: +");
  tft.setTextColor(ST77XX_YELLOW);
  tft.print(config.warnOffset, 0);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("C  Crit: +");
  tft.setTextColor(ST77XX_RED);
  tft.print(config.critOffset, 0);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("C");
  y += lineHeight;

  // Active TX
  tft.setCursor(10, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Active TX: ");
  tft.setTextColor(ST77XX_CYAN);
  tft.print(countActiveTx());
  y += lineHeight;

  // Device ID
  tft.setCursor(10, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("ID: ");
  tft.setTextColor(ST77XX_GREEN);
  tft.print(deviceID);
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
