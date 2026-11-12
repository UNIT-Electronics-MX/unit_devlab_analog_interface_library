/**
 * @file linearSensor.ino
 * @brief Convert the voltage of a linear analog sensor into physical units with
 *        a gain and an offset (value = volts * GAIN + OFFSET).
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

// Any sensor with a linear output fits value = volts * GAIN + OFFSET.
// Default: LM35 temperature sensor, 10 mV per degree C -> 100 degrees C per volt.
//
//   Sensor                       GAIN       OFFSET   UNIT
//   LM35                         100.0      0.0      "C"
//   MCP9700 (500 mV at 0 C)      100.0     -50.0     "C"
//   Potentiometer, 0-100 %       100.0/3.3  0.0      "%"  (divide by the supply)
constexpr float GAIN = 100.0f;
constexpr float OFFSET = 0.0f;
constexpr const char *UNIT = "C";

constexpr uint16_t SUPPLY_MV = 3300U;                       //Adjust the supply voltage to your sensor (mV)

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
}

void loop() {
  if (!analog.isConnected()) return;

  float volts = 0.0f;
  if (!analog.readVolts(volts)) {
    Serial.println("ERROR: read failed");
    delay(500U);
    return;
  }

  Serial.print(volts, 3);
  Serial.print(" V -> ");
  Serial.print(volts * GAIN + OFFSET, 2);
  Serial.print(' ');
  Serial.println(UNIT);
  delay(500U);
}
