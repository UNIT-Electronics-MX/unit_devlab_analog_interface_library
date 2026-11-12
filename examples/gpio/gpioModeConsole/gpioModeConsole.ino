/**
 * @file gpioModeConsole.ino
 * @brief Serial console to try every PB5 mode: input, output low/high and the
 *        two follow modes, and to read the pin and the current mode.
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

const char *modeName(DevLab_AnalogGpioMode mode) {
  switch (mode) {
    case DevLab_AnalogGpioMode::Input: return "input (pull-up)";
    case DevLab_AnalogGpioMode::OutputLow: return "output low";
    case DevLab_AnalogGpioMode::OutputHigh: return "output high";
    case DevLab_AnalogGpioMode::FollowThreshold: return "follows threshold";
    case DevLab_AnalogGpioMode::FollowThresholdInverted: return "follows threshold (inverted)";
  }
  return "unknown";
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  i  input with pull-up");
  Serial.println("  l  output low");
  Serial.println("  h  output high");
  Serial.println("  f  output follows the threshold");
  Serial.println("  n  output follows the threshold, inverted");
  Serial.println("  g  read the PB5 level");
  Serial.println("  m  show the current mode");
}

void setMode(DevLab_AnalogGpioMode mode) {
  if (analog.setGpioMode(mode)) {
    Serial.print("PB5 mode: ");
    Serial.println(modeName(mode));
  } else {
    Serial.println("ERROR: mode change failed");
  }
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
  analog.printInfo(Serial);
  printHelp();
}

void loop() {
  if (!analog.isConnected() || !Serial.available()) return;

  switch (Serial.read()) {
    case 'i': setMode(DevLab_AnalogGpioMode::Input); break;
    case 'l': setMode(DevLab_AnalogGpioMode::OutputLow); break;
    case 'h': setMode(DevLab_AnalogGpioMode::OutputHigh); break;
    case 'f': setMode(DevLab_AnalogGpioMode::FollowThreshold); break;
    case 'n': setMode(DevLab_AnalogGpioMode::FollowThresholdInverted); break;
    case 'g': {
      bool level = false;
      if (analog.readGpio(level)) {
        Serial.print("PB5 = ");
        Serial.println(level ? "HIGH" : "LOW");
      } else {
        Serial.println("ERROR: GPIO read failed");
      }
      break;
    }
    case 'm': {
      DevLab_AnalogGpioMode mode;
      if (analog.readGpioMode(mode)) {
        Serial.print("PB5 mode: ");
        Serial.println(modeName(mode));
      } else {
        Serial.println("ERROR: mode read failed");
      }
      break;
    }
    case '\r':
    case '\n': break;
    default: printHelp(); break;
  }
}
