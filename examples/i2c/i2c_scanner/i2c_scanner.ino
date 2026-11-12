/**
 * @file i2c_scanner.ino
 * @brief List every address that acknowledges on the bus and mark the UE0177
 *        analog interface (Device ID 0x0119, factory address 0x39).
 *
 * Compatible with ESP32, RP2040/RP2350, STM32 and AVR.
 *
 * @author Jonathan Mejorado
 * @organization UNIT Electronics MX
 */

#include <Arduino.h>
#include <Wire.h>
#include <DevLab_AnalogInterface.h>
#include <DevLabDDP.h>
#include <DevLab_I2C_Orchestrator.h>
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

DevLab_I2C_Orchestrator bus(I2C_BUS, I2C_FREQ);
DevLabDDP::Master master(bus, DevLab_AnalogInterface::DEVICE_ID);

void scanBus() {
  bool found = false;
  Serial.println("Address  Device");
  for (uint8_t address = 0x08U; address <= 0x77U; ++address) {
    if (!master.ping(address)) continue;
    found = true;
    Serial.print("0x");
    if (address < 0x10U) Serial.print('0');
    Serial.print(address, HEX);
    Serial.print("     ");

    DevLabDDP::DeviceInfo info;
    if (!master.identify(address, info)) {
      Serial.println("non-DDP");
    } else {
      Serial.print(DevLabDDP::deviceName(info.deviceId));
      if (info.deviceId == DevLab_AnalogInterface::DEVICE_ID) Serial.print("  <-- UE0177");
      Serial.println();
    }
  }
  if (!found) Serial.println("--       none");
}

void setup() {
  Serial.begin(115200);
  // USB boards (ESP32-S3, RP2040/RP2350): wait up to 3 s for the Serial Monitor.
  while (!Serial && millis() < 3000UL) delay(10);
  delay(200);
  bus.beginRecovered(SDA_PIN, SCL_PIN);
  scanBus();
}

void loop() {
  delay(5000U);
  scanBus();
}
