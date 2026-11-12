/**
 * @file laserKY008.ino
 * @brief Switch a KY-008 laser module on and off with the UE0177 GPIO line (PB5).
 *
 * The KY-008 is an output, not a sensor: the ADC is not used. PB5 is the only
 * line that can drive it. On the module checked for this example (pull-up from
 * VCC to S) the laser is active HIGH:
 *   S high -> laser on     S low -> laser off
 *   S left floating -> a very dim red glow through the pull-up (not a laser spot)
 *
 * Wiring: S -> PB5 (JP2 pin 3, net GPIO), middle pin -> VSYS, - -> GND.
 * Check the labels of your module before powering it: modules differ.
 *
 * CURRENT: the laser current is limited only by the resistor of the module and by
 * the supply, and PB5 is a microcontroller pin, not a power output. If the spot
 * is weak or the pin gets warm, do not drive S from PB5 directly. Use a small
 * N-channel MOSFET or an NPN transistor with PB5 on its gate/base, switching the
 * laser supply, and a 10 kOhm resistor from the gate to GND so the laser stays
 * off while the node is resetting (PB5 starts as an input with pull-up).
 *
 * SAFETY: the KY-008 is a class 3R laser (about 5 mW). Never point it at an eye,
 * yours or anyone's, nor at mirrors or shiny surfaces.
 *
 * Serial commands: 1 = on, 0 = off, b = blink (default).
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

// true: laser on when PB5 is HIGH (the module checked has a pull-up on S).
// false: a module that lights with S low; the sketch then inverts the level.
constexpr bool LASER_ON_WHEN_HIGH = true;

constexpr uint32_t BLINK_MS = 1000UL;   // on and off time in blink mode

enum class Mode : uint8_t { Off, On, Blink };

DevLab_AnalogInterface analog(I2C_BUS, DevLab_AnalogInterface::DEFAULT_ADDRESS, I2C_FREQ);

Mode mode = Mode::Blink;
bool laserOn = false;
uint32_t lastToggleMs = 0UL;

// Sets the laser and reports it; the node is told the level of PB5.
bool setLaser(bool on) {
  if (!analog.gpioWrite(on == LASER_ON_WHEN_HIGH)) {
    Serial.println("ERROR: GPIO write failed");
    return false;
  }
  laserOn = on;
  Serial.print("Laser ");
  Serial.println(on ? "ON" : "OFF");
  return true;
}

void setup() {
  Serial.begin(115200);
  // USB boards (ESP32-S3, RP2040/RP2350): wait up to 3 s for the Serial Monitor.
  while (!Serial && millis() < 3000UL) delay(10);
  delay(200);

  // PB5 as an output that starts with the laser off.
  DevLab_AnalogGpioMode off = LASER_ON_WHEN_HIGH ? DevLab_AnalogGpioMode::OutputLow
                                                 : DevLab_AnalogGpioMode::OutputHigh;
  if (!analog.beginRecovered(SDA_PIN, SCL_PIN, off)) {
    Serial.println("UE0177 module not found or PB5 could not be configured");
    return;
  }
  Serial.println("KY-008 laser on PB5 (JP2 pin 3). Do not point it at anyone's eyes.");
  Serial.println("Commands: 1 = on, 0 = off, b = blink");
  lastToggleMs = millis();
}

void loop() {
  if (!analog.isConnected()) return;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '1') {
      mode = Mode::On;
      setLaser(true);
    } else if (c == '0') {
      mode = Mode::Off;
      setLaser(false);
    } else if (c == 'b' || c == 'B') {
      mode = Mode::Blink;
      lastToggleMs = millis() - BLINK_MS;   // toggle at once
      Serial.println("Blink mode");
    }
  }

  if (mode == Mode::Blink && (uint32_t)(millis() - lastToggleMs) >= BLINK_MS) {
    lastToggleMs = millis();
    setLaser(!laserOn);
  }
}
