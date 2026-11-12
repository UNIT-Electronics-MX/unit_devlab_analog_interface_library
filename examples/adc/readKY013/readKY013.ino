/**
 * @file readKY013.ino
 * @brief Read a KY-013 analog temperature sensor (10 kOhm NTC thermistor in a
 *        voltage divider) through the UE0177 and print the temperature.
 *
 * Example output (room at rest, then a finger on the thermistor):
 *   count=1914.9  R=8783 ohm  T=28.0 C (82.5 F)
 *   count=1742.9  R=7410 ohm  T=32.5 C (90.6 F)
 *   count=1635.0  R=6646 ohm  T=35.5 C (95.8 F)
 * Warming the thermistor lowers its resistance and the count, and the printed
 * temperature rises. If it falls instead, flip THERMISTOR_ON_VCC_SIDE.
 *
 * Compatible with ESP32, RP2040/RP2350, STM32 and AVR.
 *
 * @author Jonathan Mejorado
 * @organization UNIT Electronics MX
 */

#include <Arduino.h>
#include <Wire.h>
#include <DevLab_AnalogInterface.h>
#include <math.h>

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

// ---------------------------------------------------------------------------
// KY-013: a 10 kOhm NTC thermistor and a 10 kOhm fixed resistor form a voltage
// divider, and the signal pin S carries the middle point.
//
// Wiring: S -> SIGIN (JP2 pin 1), + -> VSYS, - -> GND (take VSYS and GND from
// J2, or from the Qwiic connector). Check the JP2 pinout on your board before
// plugging the 3-pin module straight into JP2: only pin 1 is SIGIN, and pin 3
// is the PB5 GPIO, not ground.
// ---------------------------------------------------------------------------

// Fixed resistor of the divider.
constexpr float R_FIXED = 10000.0f;

// Which side of the divider is the thermistor? With the thermistor between VCC
// and S (fixed resistor to GND) use true; with the thermistor between S and GND
// use false. If the temperature falls when you warm the sensor, flip this.
// Checked on a real KY-013: warming it with a finger lowered the count, so the
// thermistor is on the GND side.
constexpr bool THERMISTOR_ON_VCC_SIDE = false;

// Temperature model:
//   false -> Steinhart-Hart with the three coefficients below.
//   true  -> Beta equation with B_VALUE and the 10 kOhm at 25 C point.
// The coefficients below fit one particular 10 kOhm NTC; against a typical
// B=3950 part they drift 3-5 C at the ends of the range, where the Beta
// equation is closer. Compare with a reference thermometer and adjust.
constexpr bool USE_BETA_EQUATION = false;
constexpr float C1 = 1.009249522e-03f;
constexpr float C2 = 2.378405444e-04f;
constexpr float C3 = 2.019202697e-07f;
constexpr float B_VALUE = 3950.0f;
constexpr float R_NOMINAL = 10000.0f;   // thermistor resistance at 25 C
constexpr float T_NOMINAL_K = 298.15f;  // 25 C in kelvin

// Readings averaged per printed value. The count is a fraction of the supply,
// so the result does not depend on whether the module runs at 3.3 V or 5 V.
constexpr uint8_t SAMPLES = 16U;

DevLab_AnalogInterface analog(I2C_BUS, DevLab_AnalogInterface::DEFAULT_ADDRESS, I2C_FREQ);

// Thermistor resistance in ohms from the 12-bit count (ratio = count / 4095).
float thermistorOhms(float count) {
  float ratio = count / (float)DevLab_AnalogInterface::ADC_MAX;
  return THERMISTOR_ON_VCC_SIDE ? R_FIXED * (1.0f / ratio - 1.0f)
                                : R_FIXED * ratio / (1.0f - ratio);
}

float temperatureCelsius(float ohms) {
  float kelvin;
  if (USE_BETA_EQUATION) {
    kelvin = 1.0f / (1.0f / T_NOMINAL_K + logf(ohms / R_NOMINAL) / B_VALUE);
  } else {
    float logR = logf(ohms);
    kelvin = 1.0f / (C1 + C2 * logR + C3 * logR * logR * logR);
  }
  return kelvin - 273.15f;
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
  Serial.println("KY-013 temperature sensor on SIGIN");
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
  float count = sum / (float)SAMPLES;

  // Near 0 or full scale the divider is open or shorted, not a temperature.
  if (count < 10.0f || count > DevLab_AnalogInterface::ADC_MAX - 10.0f) {
    Serial.print("count=");
    Serial.print(count, 1);
    Serial.println("  sensor disconnected or shorted");
    delay(500U);
    return;
  }

  float ohms = thermistorOhms(count);
  float celsius = temperatureCelsius(ohms);
  Serial.print("count=");
  Serial.print(count, 1);
  Serial.print("  R=");
  Serial.print(ohms, 0);
  Serial.print(" ohm  T=");
  Serial.print(celsius, 1);
  Serial.print(" C (");
  Serial.print(celsius * 9.0f / 5.0f + 32.0f, 1);
  Serial.println(" F)");
  delay(500U);
}
