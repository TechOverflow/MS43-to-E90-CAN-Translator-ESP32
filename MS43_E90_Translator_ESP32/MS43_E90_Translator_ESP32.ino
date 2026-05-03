/**
 * MS43-to-E90 CAN Bus Translator
 *
 * Ported from Teensy / FlexCAN_T4 to ESP32 / TechOverflow esp32_can library.
 *
 * Original author: pstrzoda
 * https://github.com/pstrzoda/MS43-to-E90-CanBus-Translator
 *
 * Port notes
 * ──────────
 * FlexCAN_T4  →  esp32_can mapping:
 *   CAN_message_t          →  CAN_FRAME
 *   msg.buf[n]             →  frame.data.byte[n]
 *   msg.id                 →  frame.id
 *   msg.len                →  frame.length
 *   can1.write(msg)        →  CAN0.sendFrame(frame)
 *   can2.write(msg)        →  CAN1.sendFrame(frame)
 *   can1.read(msg)         →  CAN0.read(frame)
 *   can2.read(msg)         →  CAN1.read(frame)
 *   can1.begin() + setBaud →  CAN0.setCANPins() + CAN0.begin()
 *
 * Pin assignments for CANipulator (ESP32-C6) — adjust to your ESP32 board:
 *   CAN0 (car bus / E90 side)   RX = GPIO16, TX = GPIO17
 *   CAN1 (DME bus / MS43 side)  RX = GPIO18, TX = GPIO19
 *
 *   Digital inputs:
 *     BATTERY_LIGHT_INPUT = GPIO4   (INPUT_PULLUP, LOW = light on)
 *     OIL_PRESS_LOW_INPUT = GPIO5   (INPUT_PULLUP, LOW = low pressure)
 *     KL15_INPUT          = GPIO6   (INPUT_PULLUP, LOW = ignition on)
 *   Digital outputs:
 *     FAN_CONTROL_PIN     = GPIO7
 *     STATUS_LED          = GPIO2   (on-board LED on most ESP32 boards)
 *
 * bitRead() is available in the ESP32 Arduino core — no change needed.
 * memcpy() / memset() are standard C — no change needed.
 */

#include <Arduino.h>
#include <SmartLeds.h>
#include <esp32_can.h>
#if defined(CONFIG_IDF_TARGET_ESP32C5)
  #include <PCA9536D.h>
#endif

// ── Pin definitions ───────────────────────────────────────────────────────────
#if defined(CONFIG_IDF_TARGET_ESP32C6)
  #define CAN0_RX_PIN         GPIO_NUM_16   // Car / E90 bus RX
  #define CAN0_TX_PIN         GPIO_NUM_17   // Car / E90 bus TX
  #define CAN1_RX_PIN         GPIO_NUM_18   // DME / MS43 bus RX
  #define CAN1_TX_PIN         GPIO_NUM_19   // DME / MS43 bus TX
  #define STATUS_LED 8
  #define LED_COUNT 1
  #define BATTERY_LIGHT_INPUT 4
  #define OIL_PRESS_LOW_INPUT 5
  #define FAN_CONTROL_PIN     10
  #define KL15_INPUT          15
#else
  #define CAN0_RX_PIN         GPIO_NUM_4    // Car / E90 bus RX
  #define CAN0_TX_PIN         GPIO_NUM_5    // Car / E90 bus TX
  #define CAN1_RX_PIN         GPIO_NUM_6    // DME / MS43 bus RX
  #define CAN1_TX_PIN         GPIO_NUM_7    // DME / MS43 bus TX
  #define STATUS_LED 2
  #define LED_COUNT 2
  #define BATTERY_LIGHT_INPUT 8
  #define OIL_PRESS_LOW_INPUT 9
  #define FAN_CONTROL_PIN     10
  #define KL15_INPUT          3
#endif

// ── Thresholds ────────────────────────────────────────────────────────────────
#define OVERHEAT_THRESHOLD      108
#define SAFE_TEMP_THRESHOLD     90
#define FAN_ON_TEMP_THRESHOLD   105
#define FAN_OFF_TEMP_THRESHOLD  97
#define CAN1_TIMEOUT_PERIOD     500   // ms — DME bus timeout
#define CAN0_TIMEOUT_PERIOD     500   // ms — Car bus timeout

// ── Byte helpers (same macros as original) ────────────────────────────────────
#define lo8(x) (uint8_t)((x) & 0xFF)
#define hi8(x) (uint8_t)(((x) >> 8) & 0xFF)

// ── Error state ───────────────────────────────────────────────────────────────
typedef struct {
    uint8_t       errorCode;
    bool          showError;
    unsigned long lastSentTime;
} ErrorState;

#define NUM_ERRORS 16
ErrorState errorStates[NUM_ERRORS];

// ── Static frame tables ───────────────────────────────────────────────────────
// Unchanged from original.

const uint8_t staticData0AA[][8] = {
    { 0xE0, 0x29, 0x02, 0x00, 0xFC, 0x0A, 0x80, 0x83 },
    { 0xE1, 0x2A, 0x02, 0x00, 0xFC, 0x0A, 0x80, 0x83 },
    { 0xDA, 0x2B, 0x02, 0x00, 0xF4, 0x0A, 0x80, 0x83 },
    { 0xF3, 0x4C, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
    { 0xF8, 0x4D, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
    { 0xF9, 0x4E, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
    { 0xEB, 0x40, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
    { 0xE4, 0x41, 0x02, 0x00, 0xE8, 0x0A, 0x80, 0x83 },
    { 0xE5, 0x42, 0x02, 0x00, 0xE8, 0x0A, 0x80, 0x83 },
    { 0x03, 0x63, 0x02, 0x00, 0xE4, 0x0A, 0x80, 0x83 },
    { 0x0C, 0x64, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
    { 0xFC, 0x55, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
    { 0xFD, 0x56, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
    { 0xFA, 0x57, 0x02, 0x00, 0xE8, 0x0A, 0x80, 0x83 },
    { 0xFF, 0x58, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
    { 0x00, 0x59, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
    { 0x0A, 0x5A, 0x02, 0x00, 0xF4, 0x0A, 0x80, 0x83 },
    { 0xF6, 0x4B, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
    { 0xF7, 0x4C, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
    { 0xF0, 0x4D, 0x02, 0x00, 0xE8, 0x0A, 0x80, 0x83 },
    { 0xF5, 0x4E, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
    { 0xF7, 0x50, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
    { 0x00, 0x51, 0x02, 0x00, 0xF4, 0x0A, 0x80, 0x83 },
    { 0xF1, 0x42, 0x02, 0x00, 0xF4, 0x0A, 0x80, 0x83 },
    { 0xEE, 0x43, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
    { 0xEF, 0x44, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
    { 0xF0, 0x45, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
    { 0xF5, 0x46, 0x02, 0x00, 0xF4, 0x0A, 0x80, 0x83 },
    { 0xF6, 0x47, 0x02, 0x00, 0xF4, 0x0A, 0x80, 0x83 },
    { 0xF3, 0x48, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
    { 0xEC, 0x49, 0x02, 0x00, 0xE8, 0x0A, 0x80, 0x83 },
};

const uint8_t staticData0A8[][8] = {
    { 0xD5, 0xEE, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xC7, 0xE0, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xC8, 0xE1, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xC9, 0xE2, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xCA, 0xE3, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xCB, 0xE4, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xCC, 0xE5, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xDD, 0xE6, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xDE, 0xE7, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xCF, 0xE8, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xB0, 0xD9, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xB1, 0xDA, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xB2, 0xDB, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xB3, 0xDC, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xB4, 0xDD, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xB5, 0xDE, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xB6, 0xDF, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xE7, 0xF0, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xE8, 0xF1, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xE9, 0xF2, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xEA, 0xF3, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xEB, 0xF4, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xEC, 0xF5, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xED, 0xF6, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xEE, 0xF7, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xEF, 0xF8, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xF0, 0xF9, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xF1, 0xFA, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xF2, 0xFB, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xF3, 0xFC, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
    { 0xF4, 0xFD, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
};

const uint8_t staticData0A9[][8] = {
    { 0x03, 0x1D, 0x7C, 0x8F, 0x2D, 0x7C, 0x6F, 0x17 },
    { 0x24, 0x1E, 0x7C, 0x8F, 0x2D, 0x7C, 0x8F, 0x17 },
    { 0x06, 0x10, 0x7C, 0x7F, 0x2D, 0x7C, 0x8F, 0x17 },
    { 0x67, 0x11, 0x7C, 0x7F, 0x2D, 0x7C, 0xEF, 0x17 },
    { 0x58, 0x12, 0x7C, 0x9F, 0x2D, 0x7C, 0xBF, 0x17 },
    { 0x69, 0x13, 0x7C, 0x9F, 0x2D, 0x7C, 0xCF, 0x17 },
    { 0x5A, 0x14, 0x7C, 0x8F, 0x2D, 0x7C, 0xCF, 0x17 },
    { 0x0B, 0x15, 0x7C, 0x8F, 0x2D, 0x7C, 0x7F, 0x17 },
    { 0x0C, 0x16, 0x7C, 0x8F, 0x2D, 0x7C, 0x7F, 0x17 },
    { 0x0D, 0x17, 0x7C, 0x8F, 0x2D, 0x7C, 0x7F, 0x17 },
    { 0x1C, 0x18, 0x7B, 0x9F, 0x2D, 0x7B, 0x7F, 0x17 },
    { 0x1D, 0x19, 0x7B, 0x9F, 0x2D, 0x7B, 0x7F, 0x17 },
    { 0x0E, 0x1A, 0x7B, 0x8F, 0x2D, 0x7B, 0x7F, 0x17 },
    { 0xEE, 0x1B, 0x7B, 0x8F, 0x2D, 0x7B, 0x5F, 0x17 },
    { 0xEF, 0x1C, 0x7B, 0x8F, 0x2D, 0x7B, 0x5F, 0x17 },
    { 0xF0, 0x1D, 0x7B, 0x8F, 0x2D, 0x7B, 0x5F, 0x17 },
    { 0xF1, 0x1E, 0x7B, 0x8F, 0x2D, 0x7B, 0x5F, 0x17 },
    { 0xD3, 0x10, 0x7B, 0x8F, 0x2D, 0x7B, 0x4F, 0x17 },
    { 0xB4, 0x11, 0x7B, 0x8F, 0x2D, 0x7B, 0x2F, 0x17 },
    { 0xB5, 0x12, 0x7B, 0x8F, 0x2D, 0x7B, 0x2F, 0x17 },
    { 0xB8, 0x13, 0x7C, 0x8F, 0x2D, 0x7C, 0x2F, 0x17 },
    { 0xA9, 0x14, 0x7C, 0x8F, 0x2D, 0x7C, 0x1F, 0x17 },
    { 0xDA, 0x15, 0x7C, 0xAF, 0x2D, 0x7C, 0x2F, 0x17 },
    { 0x9B, 0x16, 0x7C, 0xAF, 0x2D, 0x7C, 0xEF, 0x16 },
    { 0x8C, 0x17, 0x7C, 0x9F, 0x2D, 0x7C, 0xEF, 0x16 },
    { 0x8D, 0x18, 0x7C, 0x9F, 0x2D, 0x7C, 0xEF, 0x16 },
    { 0xAE, 0x19, 0x7C, 0xBF, 0x2D, 0x7C, 0xEF, 0x16 },
    { 0x60, 0x1A, 0x7C, 0xBF, 0x2D, 0x7C, 0x9F, 0x17 },
    { 0x71, 0x1B, 0x7C, 0xBF, 0x2D, 0x7C, 0xAF, 0x17 },
    { 0x72, 0x1C, 0x7C, 0xBF, 0x2D, 0x7C, 0xAF, 0x17 },
    { 0x53, 0x1D, 0x7C, 0x9F, 0x2D, 0x7C, 0xAF, 0x17 },
};

// ── Alive counters ────────────────────────────────────────────────────────────
uint8_t ALIV_TORQ_1_DME = 8;
uint8_t ALIV_TORQ_2_DME = 7;
uint8_t ALIV_TORQ_3_DME = 7;
int     ALIV_COUNT_DME  = 7;

// ── Static frame indices ──────────────────────────────────────────────────────
int index0AA = 0;
int index0A8 = 0;
int index0A9 = 0;

// ── Message timers ────────────────────────────────────────────────────────────
uint32_t MessageTimer0A8       = 20;
uint32_t MessageTimer0A9       = 20;
uint32_t MessageTimer0AA       = 20;
uint32_t MessageTimer1D0       = 20;
uint32_t MessageTimer3B4       = 28;
uint32_t DMEMessageTimer153    = 0;
uint32_t MessageTimerStatusDme = 0;

unsigned long lastDmeMessageTime = 0;
unsigned long lastCarMessageTime = 0;

// ── Signal variables ──────────────────────────────────────────────────────────
int     aRPM = 0, bRPM = 0, RPM_ENG = 0;
int     s_waterTemp               = 0;
int     s_oilTemp                 = 0;
uint8_t s_idleRegulatorState      = 0;
int     s_intakeTemp              = 0;
int     s_exhaustTemp             = 0;
int     s_speed                   = 0;
bool    s_checkEngine             = false;
double  s_batteryVolts            = 14.0;
bool    s_emlLight                = false;
bool    s_waterOverheat           = false;
bool    s_lowOilPressureFromCan   = false;
bool    s_lowOilLevel             = false;
bool    s_batteryLight            = false;
bool    s_isEngineRunning         = false;
uint8_t s_isEngineRunningCan      = 0;
bool    s_clutchDepressed         = false;
bool    s_brakeDepressed          = false;
bool    s_terminal15detected      = false;
double  s_engineTorque            = 0.0;
int     s_ambientPressure         = 0;
double  s_throttlePosition        = 0.0;

// ── Fan state ─────────────────────────────────────────────────────────────────
bool isFanOn = false;

// ── LED blink state ───────────────────────────────────────────────────────────
bool          ledState       = false;
unsigned long previousMillis = 0;

// ── Shared CAN frame buffers ──────────────────────────────────────────────────
// Replaces Teensy's single shared CAN_message_t instances.
// Using two separate frames (one per bus direction) avoids any race if the
// send and receive paths are ever called in quick succession.
CAN_FRAME carFrame;   // frames sent to car bus  (CAN0)
CAN_FRAME dmeFrame;   // frames received from DME (CAN1)

// ── Scratch helper: zero and set ID/length on carFrame ────────────────────────
static inline void prepareCarFrame(uint32_t id, uint8_t len = 8)
{
    memset(carFrame.data.byte, 0x00, 8);
    carFrame.id       = id;
    carFrame.length   = len;
    carFrame.extended = false;
    carFrame.rtr      = 0;
}

// ── LEDs & expander ──────────────────────────────────────────────────────────
SmartLed leds(LED_WS2812B, LED_COUNT, STATUS_LED, 0, DoubleBuffer);
#if defined(CONFIG_IDF_TARGET_ESP32C5)
    PCA9536 io;
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  Forward declarations
// ─────────────────────────────────────────────────────────────────────────────
void dmeCanRead();
void carCanRead();
void carCanSend();
void manageErrorMessages();
void setErrorState(uint8_t errorCode, bool showError);
uint8_t calculateChecksum(uint8_t *data, uint8_t len);
void sendMessage0AA();
void sendMessage0A8();
void sendMessage0A9();
void sendMessage1D0();
void sendMessage592(uint8_t errorCode, bool showError);
void sendMessage3B4();
void sendStatusToDme();
void sendDMEMessage153();
void controlFan();
bool isDmeCanActive();
bool isCarCanActive();
void setStatusLED();

// ─────────────────────────────────────────────────────────────────────────────
//  Checksum  (unchanged from original)
// ─────────────────────────────────────────────────────────────────────────────
uint8_t calculateChecksum(uint8_t *data, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i = 1; i <= 7; i++) sum += data[i];
    uint16_t calc = ((sum - 595) % 256) - 1;
    if (calc == 2 || calc == 11 || calc == 9) calc++;
    return (uint8_t)calc;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Fan control  (unchanged logic)
// ─────────────────────────────────────────────────────────────────────────────
void controlFan()
{
    unsigned long currentTime = millis();

    // Fail-safe: if DME has gone silent while ignition is on, run the fan
    if (currentTime - lastDmeMessageTime > CAN1_TIMEOUT_PERIOD &&
        digitalRead(KL15_INPUT) == HIGH)
    {
        digitalWrite(FAN_CONTROL_PIN, HIGH);
        isFanOn = true;
        return;
    }

    if (s_waterTemp >= FAN_ON_TEMP_THRESHOLD && !isFanOn) {
        digitalWrite(FAN_CONTROL_PIN, HIGH);
        isFanOn = true;
    } else if (s_waterTemp <= FAN_OFF_TEMP_THRESHOLD && isFanOn) {
        digitalWrite(FAN_CONTROL_PIN, LOW);
        isFanOn = false;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Error state management  (unchanged logic)
// ─────────────────────────────────────────────────────────────────────────────
void setErrorState(uint8_t errorCode, bool showError)
{
    for (int i = 0; i < NUM_ERRORS; i++) {
        if (errorStates[i].errorCode == errorCode) {
            errorStates[i].showError    = showError;
            errorStates[i].lastSentTime = millis();
            return;
        }
    }
    for (int i = 0; i < NUM_ERRORS; i++) {
        if (errorStates[i].errorCode == 0) {
            errorStates[i].errorCode    = errorCode;
            errorStates[i].showError    = showError;
            errorStates[i].lastSentTime = millis();
            return;
        }
    }
}

bool isDmeCanActive()
{
    return (millis() - lastDmeMessageTime) <= CAN1_TIMEOUT_PERIOD;
}

bool isCarCanActive()
{
    return (millis() - lastCarMessageTime) <= CAN0_TIMEOUT_PERIOD;
}

void manageErrorMessages()
{
    unsigned long curTime = millis();

    if (s_waterTemp > OVERHEAT_THRESHOLD)      setErrorState(0x27, true);
    else if (s_waterTemp < SAFE_TEMP_THRESHOLD) setErrorState(0x27, false);

    setErrorState(0x1F, s_checkEngine);
    setErrorState(0x1E, s_emlLight);
    setErrorState(0xDA, s_lowOilLevel);
    setErrorState(0xDB, s_lowOilPressureFromCan);
    setErrorState(0xD4, digitalRead(OIL_PRESS_LOW_INPUT) == LOW && s_isEngineRunning);
    setErrorState(0xD5, digitalRead(BATTERY_LIGHT_INPUT) == HIGH && s_isEngineRunning);

    if (!isDmeCanActive() && digitalRead(KL15_INPUT) == LOW)
        setErrorState(0x99, true);
    else
        setErrorState(0x99, false);

    for (int i = 0; i < NUM_ERRORS; i++) {
        if (errorStates[i].errorCode == 0) continue;

        if (curTime - errorStates[i].lastSentTime >= 8000) {
            sendMessage592(errorStates[i].errorCode, errorStates[i].showError);
            errorStates[i].lastSentTime = curTime;
            if (!errorStates[i].showError) errorStates[i].errorCode = 0;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  CAN TX — car bus (CAN0)
// ─────────────────────────────────────────────────────────────────────────────

void sendMessage0AA()
{
    prepareCarFrame(0x0AA);
    memcpy(carFrame.data.byte, staticData0AA[index0AA], 8);

    uint16_t rpm_eng      = RPM_ENG / 0.25;
    carFrame.data.byte[4] = rpm_eng & 0xFF;
    carFrame.data.byte[5] = (rpm_eng >> 8) & 0xFF;
    carFrame.data.byte[0] = calculateChecksum(carFrame.data.byte, 8);

    CAN0.sendFrame(carFrame);
    index0AA      = (index0AA + 1) % (sizeof(staticData0AA) / 8);
    MessageTimer0AA = millis();
}

void sendMessage0A8()
{
    prepareCarFrame(0x0A8);
    memcpy(carFrame.data.byte, staticData0A8[index0A8], 8);

    CAN0.sendFrame(carFrame);
    index0A8      = (index0A8 + 1) % (sizeof(staticData0A8) / 8);
    MessageTimer0A8 = millis();
}

void sendMessage0A9()
{
    prepareCarFrame(0x0A9);
    memcpy(carFrame.data.byte, staticData0A9[index0A9], 8);

    CAN0.sendFrame(carFrame);
    index0A9      = (index0A9 + 1) % (sizeof(staticData0A9) / 8);
    MessageTimer0A9 = millis();
}

void sendMessage1D0()
{
    prepareCarFrame(0x1D0);

    carFrame.data.byte[0] = s_waterTemp + 48;
    carFrame.data.byte[1] = s_oilTemp   + 48;
    carFrame.data.byte[2] = ALIV_COUNT_DME;
    carFrame.data.byte[3] = 0xBF;
    carFrame.data.byte[4] = 0x43;
    carFrame.data.byte[5] = 0xC1;
    carFrame.data.byte[6] = s_clutchDepressed ? 0xCD : 0xCC;
    carFrame.data.byte[7] = 0x8C;

    CAN0.sendFrame(carFrame);
    ALIV_COUNT_DME++;
    MessageTimer1D0 = millis();
}

void sendMessage592(uint8_t errorCode, bool showError)
{
    prepareCarFrame(0x592);

    carFrame.data.byte[0] = 0x40;
    carFrame.data.byte[1] = errorCode;
    carFrame.data.byte[2] = 0x00;
    carFrame.data.byte[3] = showError ? 0x31 : 0x30;
    carFrame.data.byte[4] = 0xFF;
    carFrame.data.byte[5] = 0xFF;
    carFrame.data.byte[6] = 0xFF;
    carFrame.data.byte[7] = 0xFF;

    CAN0.sendFrame(carFrame);
}

void sendMessage3B4()
{
    prepareCarFrame(0x3B4);

    uint16_t u_bt_scaled  = (s_batteryVolts * 68) + 0xF000;
    carFrame.data.byte[0] = u_bt_scaled & 0xFF;
    carFrame.data.byte[1] = (u_bt_scaled >> 8) & 0xFF;
    carFrame.data.byte[2] = s_isEngineRunningCan ? 0x00 : 0x09;
    carFrame.data.byte[3] = 0xFC;
    carFrame.data.byte[4] = 0xFF;
    carFrame.data.byte[5] = 0xFF;
    carFrame.data.byte[6] = 0xFF;
    carFrame.data.byte[7] = 0xFF;

    CAN0.sendFrame(carFrame);
    MessageTimer3B4 = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
//  CAN TX — DME bus (CAN1)
// ─────────────────────────────────────────────────────────────────────────────

void sendStatusToDme()
{
    uint16_t tempRpm = 12345;
    uint16_t tempMin = 3;

    // Note: this frame goes TO the DME on CAN1, not to the car bus.
    CAN_FRAME f;
    memset(f.data.byte, 0x00, 8);
    f.id           = 0x613;
    f.length       = 8;
    f.extended     = false;
    f.rtr          = 0;
    f.data.byte[0] = lo8(tempRpm);
    f.data.byte[1] = hi8(tempRpm);
    f.data.byte[2] = 0x39;
    f.data.byte[3] = lo8(tempMin);
    f.data.byte[4] = hi8(tempMin);
    // bytes 5-7 remain 0x00

    CAN1.sendFrame(f);
    MessageTimerStatusDme = millis();
}

void sendDMEMessage153()
{
    static uint8_t ascAliveCounter = 0x00;

    CAN_FRAME f;
    memset(f.data.byte, 0x00, 8);
    f.id       = 0x153;
    f.length   = 8;
    f.extended = false;
    f.rtr      = 0;

    int      speed    = 44;
    uint16_t speedRaw = speed * 16;
    uint8_t  speedLSB = speedRaw & 0x1F;
    uint8_t  speedMSB = (speedRaw >> 5) & 0xFF;

    f.data.byte[0] = 0x00;
    f.data.byte[1] = (speedLSB << 3) & 0xF8;
    f.data.byte[2] = speedMSB;
    f.data.byte[3] = 0x00;
    f.data.byte[4] = 0x00;
    f.data.byte[5] = 0x00;
    f.data.byte[6] = 0x00;
    f.data.byte[7] = ascAliveCounter;

    ascAliveCounter = (ascAliveCounter + 1) & 0x0F;

    // 0x153 is an ASC message sent on the car bus in the original
    CAN0.sendFrame(f);
}

// ─────────────────────────────────────────────────────────────────────────────
//  DME receive (CAN1) + status TX
// ─────────────────────────────────────────────────────────────────────────────
void dmeCanRead()
{
    // Drain all waiting frames before returning — on a busy bus multiple
    // frames may arrive between loop() iterations.
    if (CAN1.available()) {
        CAN1.read(dmeFrame);
        lastDmeMessageTime = millis();

        switch (dmeFrame.id) {
            case 0x316:
                aRPM             = dmeFrame.data.byte[3] * 256;
                bRPM             = aRPM + dmeFrame.data.byte[2];
                RPM_ENG          = bRPM * 0.15625;
                s_engineTorque   = dmeFrame.data.byte[4] * 0.39;
                s_isEngineRunning = RPM_ENG > 500;
                break;

            case 0x329:
                s_waterTemp          = (dmeFrame.data.byte[1] * 0.75) - 48;
                s_ambientPressure    = (dmeFrame.data.byte[2] * 2) + 598;
                s_isEngineRunningCan = bitRead(dmeFrame.data.byte[3], 3);
                s_clutchDepressed    = bitRead(dmeFrame.data.byte[3], 0);
                s_idleRegulatorState = bitRead(dmeFrame.data.byte[3], 1);
                s_throttlePosition   = dmeFrame.data.byte[5] * 0.390625;
                s_brakeDepressed     = bitRead(dmeFrame.data.byte[6], 0);
                break;

            case 0x720:
                s_intakeTemp   = dmeFrame.data.byte[1] - 48;
                s_exhaustTemp  = dmeFrame.data.byte[2] * 4;
                s_oilTemp      = dmeFrame.data.byte[3] - 48;
                s_batteryVolts = dmeFrame.data.byte[4] * 0.1;
                s_speed        = (dmeFrame.data.byte[5] * 256) + dmeFrame.data.byte[6];
                break;

            case 0x545:
                s_checkEngine         = bitRead(dmeFrame.data.byte[0], 1);
                s_emlLight            = bitRead(dmeFrame.data.byte[0], 4);
                s_lowOilLevel         = bitRead(dmeFrame.data.byte[3], 1);
                s_waterOverheat       = bitRead(dmeFrame.data.byte[3], 3);
                s_batteryLight        = bitRead(dmeFrame.data.byte[5], 0);
                s_lowOilPressureFromCan = bitRead(dmeFrame.data.byte[7], 7);
                break;

            default:
                break;
        }

        Serial.printf("Temp: %d  RPM: %d  Torque: %.2f  Running: %d\n",
                    s_waterTemp, RPM_ENG, s_engineTorque, s_isEngineRunning);
    }

    // Periodically send the status frame back to the DME
    if ((uint32_t)(millis() - MessageTimerStatusDme) >= 200) {
        sendStatusToDme();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Car receive (CAN0)
// ─────────────────────────────────────────────────────────────────────────────
void carCanRead()
{
    if (CAN0.available()) {
        CAN0.read(carFrame);
        lastCarMessageTime = millis();
        printFrame(&carFrame);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Car bus TX scheduler
// ─────────────────────────────────────────────────────────────────────────────
void carCanSend()
{
    // Don't flood the car bus if the DME hasn't been heard from yet.
    if (!isDmeCanActive()) return;

    uint32_t curTime = millis();
    if (curTime - MessageTimer0A8 >= 10) sendMessage0A8();
    if (curTime - MessageTimer0A9 >= 10) sendMessage0A9();
    if (curTime - MessageTimer0AA >= 10) sendMessage0AA();
    if (curTime - MessageTimer3B4 >= 4000) sendMessage3B4();
    manageErrorMessages();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Status LED blink
// ─────────────────────────────────────────────────────────────────────────────
void setStatusLED()
{
    unsigned long dmeActive = isDmeCanActive();
    unsigned long carActive = isCarCanActive();
    unsigned long now       = millis();
    
    leds[0] = Rgb { 
        (!dmeActive && !carActive) ? 20 : 0, 
        (dmeActive && !carActive) ? 100 : 0, 
        (dmeActive && carActive) ? 100 : 0 
    };
    leds[1] = Rgb { 0, 0, 0 };
    leds.show();
    leds.wait();
}

// ─────────────────────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    Serial.println("Starting MS43 Translator (ESP32)");

    // Initialise error state table
    for (int i = 0; i < NUM_ERRORS; i++) {
        errorStates[i] = { 0, false, 0 };
    }

    // Setup status LEDs
    leds[0] = Rgb { 20, 0, 0 };
    leds[1] = Rgb { 20, 0, 0 };
    leds.show();
    leds.wait();

    // Outputs
    pinMode(FAN_CONTROL_PIN, OUTPUT);

    // Inputs
    pinMode(BATTERY_LIGHT_INPUT, INPUT_PULLUP);
    pinMode(OIL_PRESS_LOW_INPUT, INPUT_PULLUP);
    pinMode(KL15_INPUT,          INPUT_PULLUP);

    // Set Shutdown and Standby LOW for high-speed mode
    #if defined(CONFIG_IDF_TARGET_ESP32C5)
        Wire.begin();
        // Initialize the PCA9536 with a begin function
        if (io.begin() == false) {
            Serial.println("PCA9536 not detected. Please check wiring. Freezing...");
            while (1);
        } else {
            Serial.println("PCA9536 started");
        }

        for (int i = 0; i < 4; i++) {
            io.pinMode(i, OUTPUT);
            io.write(i, LOW);
        }
    #else 
        for (int i = 0; i < 4; i++) {
            pinMode(i, OUTPUT);
            digitalWrite(i, LOW);
        }
    #endif

    // ── CAN0 — car bus / E90 side ─────────────────────────────────────────
    CAN0.setCANPins(CAN0_RX_PIN, CAN0_TX_PIN);
    CAN0.begin(100000); // 100000 for K-CAN (Body/Kombi) or 500000 for PT-CAN
    CAN0.watchFor();   // accept all IDs

    // ── CAN1 — DME bus / MS43 side ───────────────────────────────────────
    CAN1.setCANPins(CAN1_RX_PIN, CAN1_TX_PIN);
    CAN1.begin(500000);
    CAN1.watchFor();

    // Optional: uncomment for verbose driver output
    // CAN0.debuggingMode = true;
    // CAN1.debuggingMode = true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    dmeCanRead();
    // carCanRead(); // for testing only
    carCanSend();
    controlFan();
    setStatusLED();
    // CAN0.resetIfStale(3000);
    // CAN1.resetIfStale(3000);
}

void printFrame(CAN_FRAME *message) {
    Serial.print(message->id, HEX);
    if (message->extended) Serial.print(" X ");
    else Serial.print(" S [");   
    Serial.print(message->length, DEC);
    Serial.print("] ");
    for (int i = 0; i < message->length; i++) {
        Serial.print(message->data.byte[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}
