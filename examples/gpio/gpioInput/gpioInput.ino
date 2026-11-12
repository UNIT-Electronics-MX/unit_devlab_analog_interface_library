/**
 * @file gpioInput.ino
 * @brief Use PB5 (JP2 pin 3, net GPIO) as a digital input, for example to read
 *        a 3-pin digital sensor, and print every change.
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

DevLab_AnalogInterface analog(I2C_BUS, DevLab_AnalogInterface::DEFAULT_ADDRESS, I2C_FREQ);

bool lastLevel = false;
bool haveLast = false;
uint32_t lastReportMs = 0U;

void setup() {
  Serial.begin(115200);
  // USB boards (ESP32-S3, RP2040/RP2350): wait up to 3 s for the Serial Monitor.
  while (!Serial && millis() < 3000UL) delay(10);
  delay(200);
  // begin() sets PB5 as an input with a pull-up, so it reads HIGH when nothing
  // is connected and LOW when the sensor pulls the line down.
  if (!analog.beginRecovered(SDA_PIN, SCL_PIN, DevLab_AnalogGpioMode::Input)) {
    Serial.println("UE0177 module not found or PB5 could not be configured");
    return;
  }
  Serial.println("PB5 is an input. Prints every change and the level once per second.");
}

void loop() {
  if (!analog.isConnected()) return;

  bool level = false;
  if (!analog.readGpio(level)) {
    Serial.println("ERROR: GPIO read failed");
    delay(500U);
    return;
  }

  // Print at once on a change, and also once per second so the monitor shows
  // the sketch is alive while the level stays constant (HIGH with nothing
  // connected, because of the pull-up).
  bool changed = !haveLast || level != lastLevel;
  if (changed || millis() - lastReportMs >= 1000UL) {
    Serial.print(changed ? "PB5 changed -> " : "PB5 = ");
    Serial.println(level ? "HIGH" : "LOW");
    lastLevel = level;
    haveLast = true;
    lastReportMs = millis();
  }
  delay(10U);
}
