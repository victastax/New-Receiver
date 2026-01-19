1. Hardware Overview

MCU

ESP32-WROOM-32 Dev Board

Peripherals

1.9" ST7789 IPS TFT (170×320, SPI)

RA-01 LoRa module (SX1278, 433 MHz)

GY-GPS6MV2 GPS module

V474 SPI microSD card module

Passive 2-pin piezo buzzer

3 LEDs (Green / Yellow / Red)

1 push-button (acknowledge / test)

2. Final Locked Pinout (DO NOT CHANGE)
User I/O
Function	GPIO	Notes
Button	35	External 10 kΩ pull-up to 3V3, button to GND
Green LED	21	Active-LOW
Yellow LED	22	Active-LOW
Red LED	17	Active-LOW
Buzzer (+)	25	Passive piezo, PWM
Buzzer (–)	GND	
#define LED_ON  LOW
#define LED_OFF HIGH

TFT Display (ST7789 – VSPI)
Signal	GPIO
MOSI	23
SCK	18
CS	15 (10 kΩ pull-up fitted)
DC	16
RST	4
BLK	3V3 (always on)
VCC	3V3
GND	GND
SD Card (V474 – VSPI shared)
Signal	GPIO
MOSI	23
MISO	19
SCK	18
CS	27 (10 kΩ pull-up fitted)
VCC	3V3
GND	GND

⚠ SD must be initialised before TFT or SPI will lock up.

LoRa (RA-01 – HSPI)
Signal	GPIO
SCK	14
MISO	12
MOSI	13
NSS / CS	5
RESET	33
DIO0	26
VCC	3V3
GND	GND

⚠ HSPI must be instantiated globally:

SPIClass hspi(HSPI);

GPS (UART2)
Signal	GPIO
GPS TX → ESP RX	34
GPS RX ← ESP TX	32
Baud	9600
3. Electrical Design Notes (Critical)

LEDs are active-LOW

GPIO35 cannot use internal pull-ups → external resistor required

TFT CS and SD CS require pull-ups

Buzzer is passive, must be PWM driven

TFT + SD share VSPI

LoRa runs on HSPI to avoid bus contention

4. TFT_eSPI Configuration (User_Setup.h)
#define ST7789_DRIVER

#define TFT_WIDTH  170
#define TFT_HEIGHT 320

#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15
#define TFT_DC   16
#define TFT_RST  4
#define TFT_BL   -1   // tied to 3V3

#define SPI_FREQUENCY 8000000

#define TFT_COL_OFFSET 52
#define TFT_ROW_OFFSET 0

5. System States
enum SystemState {
  STATE_OK,
  STATE_WARNING,
  STATE_ALARM
};

State	LED	Buzzer
OK	Green	Off
WARNING	Yellow	Quiet, slow
ALARM	Red	Loud, fast
6. Button Behaviour

Short press cycles states in test mode

In ALARM:

Button acknowledges alarm

Buzzer silenced

Red LED remains active

Alarm clears only when conditions return safe

7. Buzzer Control (ESP32 Arduino Core 3.x)

⚠ ledcSetup() does not exist in core 3.x.

Correct usage:

ledcAttach(BUZZER_PIN, 2000, 8);
ledcWriteTone(BUZZER_PIN, frequency);
ledcWrite(BUZZER_PIN, duty);

Tuned Behaviour
Mode	Frequency	Duty	Interval
WARNING	~1800 Hz	~20%	1 s
ALARM	~3000 Hz	~70%	250 ms
8. Firmware Architecture (Modular)
firmware/
├── AxleWatch_RX.ino        ← glue only
│
├── config/                ← single source of truth
│   ├── pinout.h
│   ├── thresholds.h
│   └── system_config.h
│
├── drivers/
│   ├── leds.*
│   ├── buzzer.*
│   └── button.*
│
├── lora/
│   └── lora_rx.*
│
├── gps/
│   └── gps.*
│
├── logging/
│   └── sd_logger.*
│
└── ui/
    └── ui.*

Design rules

.ino contains no hardware logic

All pins live in pinout.h

Each peripheral has its own module

No global state outside modules

9. SPI Strategy

VSPI → TFT + SD

HSPI → LoRa

Prevents watchdog resets and bus collisions.

10. Known Pitfalls (Do Not Repeat)

❌ Initialising TFT before SD
❌ Declaring SPIClass hspi inside setup()
❌ Using GPIO35 internal pull-ups
❌ Driving passive buzzer with DC
❌ Sharing LoRa on VSPI
❌ Mixing pin definitions across files

11. Responsibilities for Next Firmware Engineer

The incoming firmware developer should:

Implement LoRa packet parsing

Map transmitter IDs → trailers

Apply alarm logic:

Absolute temp

Delta vs ambient

Rate-of-rise

Log events to SD (CSV or JSON)

Expand UI (history, per-asset view)

Optional: Wi-Fi / cloud upload

12. Project Status

✅ Hardware bring-up complete

✅ All peripherals tested

✅ Stable merged firmware base

🟡 Application logic pending


# AxleWatch Receiver (ESP32-WROOM-32)

AxleWatch RX is an ESP32-based receiver designed to monitor heavy-vehicle trailer wheel hub temperatures using LoRa, present live status on a TFT display, alert operators via LEDs and a buzzer, and log events to an SD card with optional GPS context.

This document is the single source of truth for hardware configuration and firmware architecture.

---

## 1. Hardware Overview

**MCU**
- ESP32-WROOM-32 Dev Board

**Peripherals**
- 1.9\" ST7789 IPS TFT (170×320, SPI)
- RA-01 LoRa module (SX1278, 433 MHz)
- GY-GPS6MV2 GPS module
- V474 SPI microSD card module
- Passive 2-pin piezo buzzer
- 3 LEDs (Green / Yellow / Red)
- 1 push-button (acknowledge / test)

---

## 2. Final Locked Pinout (DO NOT CHANGE)

### User I/O

| Function | GPIO | Notes |
|-------|------|------|
| Button | 35 | External 10 kΩ pull-up to 3V3, button to GND |
| Green LED | 21 | Active-LOW |
| Yellow LED | 22 | Active-LOW |
| Red LED | 17 | Active-LOW |
| Buzzer (+) | 25 | Passive piezo, PWM |
| Buzzer (–) | GND | |

```cpp
#define LED_ON  LOW
#define LED_OFF HIGH





