/**
 * @file readKY027.ino
 * @brief KY-027 mercury tilt switch ("magic light cup") on the UE0177 GPIO line
 *        (PB5): reports when the module is tilted and when it is upright again,
 *        with debounce and a counter.
 *
 * The signal of the KY-027 is a mercury switch: it only has two states, so a
 * digital input is the right way to read it, not the ADC. PB5 already has the
 * pull-up the switch needs, so no extra parts are required. As in the original
 * sketch, a tilt reads HIGH (the mercury opens the circuit) and the upright
 * position reads LOW:
 *   upright -> switch closed to GND -> PB5 LOW
 *   tilted  -> switch open          -> PB5 pulled HIGH by the node
 *
 * Wiring: S -> PB5 (JP2 pin 3, net GPIO), - -> GND (from J2 or the Qwiic
 * connector). The + pin is not needed for the switch. Check the JP2 pinout on
 * your board: PB5 is pin 3, and pin 1 is the analog SIGIN.
 *
 * The UE0177 has a single GPIO and the switch uses it, so the LED of the module
 * (pin L, driven by pin 5 in the original sketch) is not driven by the node.
 * This sketch lights the LED of the master board (LED_BUILTIN) instead, and it
 * can also drive the L pin of the module from a master pin: set MODULE_LED_PIN.
 *
 * Example output:
 *   tilt: NO    detections=0   (PB5=LOW)
 *   Tilt detected (#1)
 *   Upright again
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

// true: a tilt reads HIGH on PB5 (the mercury opens the circuit, as in the
// original sketch, where 1 meant tilted). false: a module that reads LOW when
// tilted. If the result is the other way round on your module, flip this.
constexpr bool TILT_READS_HIGH = true;

// Master pin wired to the L (LED) pin of the module, or -1 when it is not used.
// It lights the module LED while the module is tilted, like the original sketch.
constexpr int MODULE_LED_PIN = -1;

constexpr uint16_t SAMPLE_MS = 5U;       // how often PB5 is read over I2C
constexpr uint8_t DEBOUNCE_SAMPLES = 10U; // equal readings in a row to accept a change (50 ms)
constexpr uint32_t STATUS_MS = 1000UL;   // state line; the original printed every second

// ===== Debounce begin ======================================================
// A mercury switch bounces for a few milliseconds each time the drop touches or
// leaves the contacts. A change is accepted only after DEBOUNCE_SAMPLES equal
// readings in a row.
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
#endif
  if (MODULE_LED_PIN >= 0) digitalWrite(MODULE_LED_PIN, on ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  // USB boards (ESP32-S3, RP2040/RP2350): wait up to 3 s for the Serial Monitor.
  while (!Serial && millis() < 3000UL) delay(10);
  delay(200);
#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
#endif
  if (MODULE_LED_PIN >= 0) pinMode(MODULE_LED_PIN, OUTPUT);

  // PB5 as an input with the node's pull-up (the default mode).
  if (!analog.beginRecovered(SDA_PIN, SCL_PIN, DevLab_AnalogGpioMode::Input)) {
    Serial.println("UE0177 module not found or PB5 could not be configured");
    return;
  }
  Serial.println("KY-027 tilt switch on PB5 (JP2 pin 3). Tilt the module...");
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
  bool tilted = TILT_READS_HIGH ? level : !level;

  if (!started) {
    debounce.reset(tilted);   // start from the current position, not from "upright"
    showLed(tilted);
    started = true;
  } else if (debounce.update(tilted)) {
    showLed(debounce.state());
    if (debounce.state()) {
      ++detections;
      Serial.print("Tilt detected (#");
      Serial.print(detections);
      Serial.println(")");
    } else {
      Serial.println("Upright again");
    }
  }

  if ((uint32_t)(now - lastStatusMs) >= STATUS_MS) {
    lastStatusMs = now;
    Serial.print("tilt: ");
    Serial.print(debounce.state() ? "YES" : "NO ");
    Serial.print("   detections=");
    Serial.print(detections);
    Serial.print("   (PB5=");
    Serial.print(level ? "HIGH" : "LOW");
    Serial.println(")");
  }
}
