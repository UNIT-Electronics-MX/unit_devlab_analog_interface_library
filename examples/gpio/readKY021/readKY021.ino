/**
 * @file readKY021.ino
 * @brief Magnetic reed switch KY-021 on the UE0177 GPIO line (PB5): reports when
 *        a magnet comes close and when it goes away, with debounce and a counter.
 *
 * The KY-021 is just a reed contact: open without a magnet, closed with one. A
 * digital input is the right way to read it, not the ADC, and PB5 already has
 * the pull-up the module needs, so no extra parts are required:
 *   no magnet -> contact open   -> PB5 pulled HIGH by the node
 *   magnet    -> contact closed -> PB5 pulled LOW through the reed
 *
 * Wiring: S -> PB5 (JP2 pin 3, net GPIO), - -> GND (from J2 or the Qwiic
 * connector). The + pin is not needed by the reed. Check the JP2 pinout on your
 * board: PB5 is pin 3, and pin 1 is the analog SIGIN.
 *
 * Example output:
 *   magnet: NO   detections=0   (PB5=HIGH)
 *   Magnet detected (#1)
 *   Magnet removed
 *
 * The LED of the master board (LED_BUILTIN, the LED of pin 13 in the original
 * sketch) follows the magnet when the board has one.
 *
 * Compatible with ESP32, RP2040/RP2350, STM32 and AVR.
 *
 * @author Jonathan Mejorado
 * @organization UNIT Electronics MX
 */

#include <Arduino.h>
#include <Wire.h>
#include <DevLab_AnalogInterface.h>

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
  #if defined(PIN_WIRE1_SDA) && defined(PIN_WIRE1_SCL) && \
      PIN_WIRE1_SDA == 24U && PIN_WIRE1_SCL == 25U
    // Pulsar connector pins.
    constexpr int SDA_PIN = 24;
    constexpr int SCL_PIN = 25;
  #else
    // Generic RP2040/RP2350 wiring.
    constexpr int SDA_PIN = 12;
    constexpr int SCL_PIN = 13;
  #endif
  // The RP2040/RP2350 I2C controller is fixed by the pin: SDA on GPIO 0, 4, 8,
  // ... 24 belongs to I2C0 (Wire) and GPIO 2, 6, 10, ... to I2C1 (Wire1).
  // Using the wrong object leaves the controller disconnected from the pins
  // and every transfer times out (GPIO 24/25 on the Pulsar are I2C0).
  #define I2C_BUS (((SDA_PIN / 2) % 2 == 0) ? Wire : Wire1)
#elif defined(ARDUINO_ARCH_ESP32)
  #define I2C_BUS Wire
  constexpr int SDA_PIN = 6;
  constexpr int SCL_PIN = 7;
#elif defined(ARDUINO_ARCH_STM32)
  // STM32duino: default I2C pins of the selected board.
  #define I2C_BUS Wire
  constexpr int SDA_PIN = SDA;
  constexpr int SCL_PIN = SCL;
#elif defined(ARDUINO_ARCH_AVR)
  // AVR has fixed I2C pins (Uno/Nano: A4/A5, Mega: 20/21, Leonardo: 2/3);
  // SDA/SCL come from the board variant and begin() ignores the pin numbers.
  #define I2C_BUS Wire
  constexpr int SDA_PIN = SDA;
  constexpr int SCL_PIN = SCL;
#else
#error "Use an ESP32, RP2040, RP2350, STM32, or AVR master"
#endif

// The UE0177 firmware runs its I2C slave at 100 kHz.
constexpr uint32_t I2C_FREQ = 100000UL;

// true: the reed connects S to GND, so a magnet reads LOW (as in the original
// sketch, which found HIGH at rest). false: a module whose contact connects S
// to the supply, so a magnet reads HIGH.
constexpr bool MAGNET_PULLS_LOW = true;

constexpr uint16_t SAMPLE_MS = 5U;      // how often PB5 is read over I2C
constexpr uint8_t DEBOUNCE_SAMPLES = 4U; // consecutive equal readings to accept a change (20 ms)
constexpr uint32_t STATUS_MS = 1000UL;  // state line; the original printed every loop

// ===== Debounce begin ======================================================
// A reed contact bounces for a few milliseconds when it closes or opens. A
// change is accepted only after DEBOUNCE_SAMPLES equal readings in a row.
class Debounce {
 public:
  void reset(bool value) {
    _state = value;
    _candidate = value;
    _count = 0U;
  }

  // Feed one reading. Returns true when the stable state has just changed.
  bool update(bool raw) {
    if (raw == _state) {
      _candidate = _state;
      _count = 0U;
      return false;
    }
    if (raw != _candidate) {
      _candidate = raw;
      _count = 0U;
    }
    if (++_count >= DEBOUNCE_SAMPLES) {
      _state = raw;
      _count = 0U;
      return true;
    }
    return false;
  }

  bool state() const { return _state; }

 private:
  bool _state = false;
  bool _candidate = false;
  uint8_t _count = 0U;
};
// ===== Debounce end ========================================================

DevLab_AnalogInterface analog(I2C_BUS, DevLab_AnalogInterface::DEFAULT_ADDRESS, I2C_FREQ);
Debounce debounce;

bool started = false;
uint16_t detections = 0U;
uint32_t lastSampleMs = 0UL;
uint32_t lastStatusMs = 0UL;

void showLed(bool on) {
#ifdef LED_BUILTIN
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

void setup() {
  Serial.begin(115200);
  // USB boards (ESP32-S3, RP2040/RP2350): wait up to 3 s for the Serial Monitor.
  while (!Serial && millis() < 3000UL) delay(10);
  delay(200);
#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
#endif

  // PB5 as an input with the node's pull-up (the default mode).
  if (!analog.beginRecovered(SDA_PIN, SCL_PIN, DevLab_AnalogGpioMode::Input)) {
    Serial.println("UE0177 module not found or PB5 could not be configured");
    return;
  }
  Serial.println("KY-021 magnetic switch on PB5 (JP2 pin 3). Bring a magnet close...");
}

void loop() {
  if (!analog.isConnected()) return;

  uint32_t now = millis();
  if ((uint32_t)(now - lastSampleMs) < SAMPLE_MS) return;
  lastSampleMs = now;

  bool level = false;
  if (!analog.readGpio(level)) {
    Serial.println("ERROR: GPIO read failed");
    delay(500U);
    return;
  }
  bool magnet = MAGNET_PULLS_LOW ? !level : level;

  if (!started) {
    debounce.reset(magnet);   // start from the current state, not from "no magnet"
    showLed(magnet);
    started = true;
  } else if (debounce.update(magnet)) {
    showLed(debounce.state());
    if (debounce.state()) {
      ++detections;
      Serial.print("Magnet detected (#");
      Serial.print(detections);
      Serial.println(")");
    } else {
      Serial.println("Magnet removed");
    }
  }

  if ((uint32_t)(now - lastStatusMs) >= STATUS_MS) {
    lastStatusMs = now;
    Serial.print("magnet: ");
    Serial.print(debounce.state() ? "YES" : "NO ");
    Serial.print("   detections=");
    Serial.print(detections);
    Serial.print("   (PB5=");
    Serial.print(level ? "HIGH" : "LOW");
    Serial.println(")");
  }
}
