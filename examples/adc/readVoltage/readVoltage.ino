/**
 * @file readVoltage.ino
 * @brief Measure the voltage at SIGIN with host-side averaging. Set the module
 *        supply (3.3 V or 5 V) so the conversion is correct.
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

// Supply (VSYS) of the UE0177 module in millivolts. The ADC count is a
// fraction of the supply, so a 5 V board only needs this value changed.
constexpr uint16_t SUPPLY_MV = 3300U;

// Readings averaged per printed value. More samples give less noise.
constexpr uint8_t SAMPLES = 16U;

DevLab_AnalogInterface analog(I2C_BUS, DevLab_AnalogInterface::DEFAULT_ADDRESS, I2C_FREQ);

void setup() {
  Serial.begin(115200);
  // USB boards (ESP32-S3, RP2040/RP2350): wait up to 3 s for the Serial Monitor.
  while (!Serial && millis() < 3000UL) delay(10);
  delay(200);
  if (!analog.beginRecovered(SDA_PIN, SCL_PIN)) {
    Serial.println("UE0177 module not found; check wiring, power and firmware");
    return;
  }
  analog.setSupplyMillivolts(SUPPLY_MV);
  Serial.print("Supply: ");
  Serial.print(analog.supplyMillivolts());
  Serial.println(" mV");
}

void loop() {
  if (!analog.isConnected()) return;

  uint32_t sum = 0U;
  for (uint8_t i = 0U; i < SAMPLES; ++i) {
    uint16_t raw = 0U;
    if (!analog.readRaw(raw)) {
      Serial.println("ERROR: read failed");
      delay(500U);
      return;
    }
    sum += raw;
    delay(2U);
  }

  // Convert the averaged count with the same scaling the library uses.
  float volts = ((sum / (float)SAMPLES) * SUPPLY_MV) / DevLab_AnalogInterface::ADC_MAX / 1000.0f;
  Serial.print("Voltage: ");
  Serial.print(volts, 3);
  Serial.println(" V");
  delay(250U);
}
