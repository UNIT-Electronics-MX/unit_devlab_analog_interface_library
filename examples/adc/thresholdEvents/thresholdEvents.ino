/**
 * @file thresholdEvents.ino
 * @brief Report when the analog signal crosses the software threshold. The
 *        node applies hysteresis and debounce, so the events are clean.
 *
 * The threshold is set here, in setup(), with setThresholdsMillivolts(): the
 * state turns ON when the signal reaches THRESHOLD_ON_MV and OFF when it falls
 * to THRESHOLD_OFF_MV. Keep OFF below ON: the gap is the hysteresis that keeps a
 * noisy signal from flickering. The levels live in the node's RAM, so they go
 * back to the firmware defaults (2300 / 1800 counts) after the node resets.
 * Needs UE0177 firmware 0.3 or newer; with 0.2 the sketch keeps the fixed
 * levels of the firmware and says so.
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

// Threshold, in millivolts at the pin (scale: SUPPLY_MV).
constexpr bool CUSTOM_THRESHOLD = true;   // false: keep the levels of the firmware
constexpr uint16_t THRESHOLD_ON_MV = 2000U;
constexpr uint16_t THRESHOLD_OFF_MV = 1500U;
constexpr uint16_t SUPPLY_MV = 3300U;     // 5000U when the module runs at 5 V

DevLab_AnalogInterface analog(I2C_BUS, DevLab_AnalogInterface::DEFAULT_ADDRESS, I2C_FREQ);

// The node compares counts (0..4095 = 0..SUPPLY_MV), not millivolts: 2300 mV at
// 3.3 V is count 2854. Printed values show both.
uint16_t countToMv(uint16_t count) {
  return (uint16_t)(((uint32_t)count * SUPPLY_MV + DevLab_AnalogInterface::ADC_MAX / 2U) /
                    DevLab_AnalogInterface::ADC_MAX);
}

void printReading(uint16_t raw) {
  Serial.print("  raw=");
  Serial.print(raw);
  Serial.print(" (");
  Serial.print(countToMv(raw));
  Serial.println(" mV)");
}

void setup() {
  Serial.begin(115200);
  // USB boards (ESP32-S3, RP2040/RP2350): wait up to 3 s for the Serial Monitor.
  while (!Serial && millis() < 3000UL) delay(10);
  delay(200);
  if (!analog.beginRecovered(SDA_PIN, SCL_PIN)) {
    Serial.println("UE0177 module not found; check wiring, power and firmware");
    return;
  }

  if (CUSTOM_THRESHOLD) {
    analog.setSupplyMillivolts(SUPPLY_MV);
    if (!analog.setThresholdsMillivolts(THRESHOLD_ON_MV, THRESHOLD_OFF_MV)) {
      Serial.println("Could not set the threshold (firmware older than 0.3, or ON <= OFF); "
                     "using the levels of the firmware");
    }
  }

  uint16_t high = 0U;
  uint16_t low = 0U;
  if (analog.readThresholds(high, low)) {
    Serial.print("Turns ON at count >= ");
    Serial.print(high);
    Serial.print(" (");
    Serial.print(countToMv(high));
    Serial.print(" mV), OFF at count <= ");
    Serial.print(low);
    Serial.print(" (");
    Serial.print(countToMv(low));
    Serial.println(" mV)");
  }
}

uint32_t lastReportMs = 0U;

void loop() {
  if (!analog.isConnected()) return;

  // poll() reads the status byte; the node clears its edge flag on that read,
  // so every crossing is reported exactly once.
  DevLab_AnalogInterface_State state;
  if (!analog.poll(state)) {
    Serial.println("ERROR: status read failed");
    delay(500U);
    return;
  }

  if (state.adcError) Serial.println("WARNING: ADC conversion failed");
  if (state.changed) {
    uint16_t raw = 0U;
    analog.readRaw(raw);
    Serial.print(state.above ? "ABOVE threshold" : "BELOW threshold");
    printReading(raw);
    lastReportMs = millis();
  } else if (millis() - lastReportMs >= 1000UL) {
    // Heartbeat so the monitor shows the sketch is alive between crossings.
    uint16_t raw = 0U;
    analog.readRaw(raw);
    Serial.print(state.above ? "above" : "below");
    printReading(raw);
    lastReportMs = millis();
  }
  delay(10U);
}
