/**
 * @file changeAddress.ino
 * @brief Change the I2C address of a UE0177 analog interface from the Serial
 *        Monitor, and go back to the factory address 0x39.
 *
 * Commands (115200 baud, any line ending):
 *   scan                          list every address on the bus
 *   change <current> <new>        e.g. "change 39 40" (hexadecimal, 08..77)
 *   factory <current>             back to 0x39, e.g. "factory 40"
 *
 * The new address is stored in the Flash of the node and the node restarts
 * on it, so it survives a power cycle. Use it to put two modules on the same
 * bus. Afterwards create the driver with the new address:
 *
 *   DevLab_AnalogInterface analog(Wire, 0x40);
 *
 * If you forget the address, "scan" finds it. Only one module should be
 * connected while you change an address, otherwise "change" may reach the
 * wrong one.
 *
 * Needs firmware 0.4 or newer on the module.
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
constexpr uint8_t ADDRESS_MIN = 0x08U;
constexpr uint8_t ADDRESS_MAX = 0x77U;
// The node restarts after storing the address; give it time to come back.
constexpr uint16_t RESTART_DELAY_MS = 100U;

DevLab_I2C_Orchestrator bus(I2C_BUS, I2C_FREQ);
DevLabDDP::Master master(bus, DevLab_AnalogInterface::DEVICE_ID);
String inputLine;

bool parseAddress(const String &text, uint8_t &address) {
  char *end = nullptr;
  long value = strtol(text.c_str(), &end, 16);
  if (end == text.c_str() || *end != '\0' ||
      value < (long)ADDRESS_MIN || value > (long)ADDRESS_MAX) {
    return false;
  }
  address = (uint8_t)value;
  return true;
}

void printHexAddress(uint8_t address) {
  Serial.print("0x");
  if (address < 0x10U) Serial.print('0');
  Serial.print(address, HEX);
}

void scanBus() {
  bool found = false;
  Serial.println("Address  Device");
  for (uint8_t address = ADDRESS_MIN; address <= ADDRESS_MAX; ++address) {
    if (!master.ping(address)) continue;
    found = true;
    printHexAddress(address);
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

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  scan");
  Serial.println("  change <current_hex> <new_hex>");
  Serial.println("  factory <current_hex>");
  Serial.println("Examples:");
  Serial.println("  change 39 40");
  Serial.println("  factory 40");
}

void changeAddress(const String &oldText, const String &newText) {
  uint8_t oldAddress, newAddress;
  if (!parseAddress(oldText, oldAddress) || !parseAddress(newText, newAddress) ||
      oldAddress == newAddress) {
    Serial.println("ERROR addresses must be different hexadecimal values from 08 to 77");
    return;
  }
  if (!master.matchesExpectedDevice(oldAddress)) {
    Serial.println("ERROR current address does not contain a UE0177 (0x0119)");
    return;
  }
  if (master.ping(newAddress)) {
    Serial.println("ERROR new address is already in use");
    return;
  }

  Serial.print("Changing ");
  printHexAddress(oldAddress);
  Serial.print(" -> ");
  printHexAddress(newAddress);
  Serial.println("...");

  // Two transactions: command 0x21, then the new address. The node stores it
  // in Flash, restarts, and setI2cAddress() checks that it answers there.
  if (!master.setI2cAddress(oldAddress, newAddress)) {
    Serial.println("ERROR address change failed");
    Serial.println("The module may need firmware 0.4 or newer; run scan to see where it is.");
    return;
  }

  Serial.print("OK the module now answers at ");
  printHexAddress(newAddress);
  Serial.println();
  scanBus();
}

void factoryReset(const String &text) {
  uint8_t address;
  if (!parseAddress(text, address)) {
    Serial.println("ERROR address must be a hexadecimal value from 08 to 77");
    return;
  }
  if (!master.matchesExpectedDevice(address)) {
    Serial.println("ERROR that address does not contain a UE0177 (0x0119)");
    return;
  }

  uint8_t response = 0U;
  if (!master.readCommand(address, CMD_RESET_FACTORY, &response, 1U, 10U) ||
      (response & 0x0FU) != RESP_FACTORY_RESET) {
    Serial.println("ERROR factory reset failed");
    return;
  }

  delay(RESTART_DELAY_MS);
  const uint8_t factory = DevLab_AnalogInterface::DEFAULT_ADDRESS;
  if (!master.matchesExpectedDevice(factory)) {
    Serial.println("ERROR the module did not come back at the factory address");
    return;
  }
  Serial.print("OK the module is back at ");
  printHexAddress(factory);
  Serial.println();
  scanBus();
}

void processCommand(String line) {
  line.trim();
  line.toLowerCase();

  if (line == "scan") {
    scanBus();
    return;
  }

  int firstSpace = line.indexOf(' ');
  String name = firstSpace < 0 ? line : line.substring(0, firstSpace);
  String rest = firstSpace < 0 ? String("") : line.substring(firstSpace + 1);
  rest.trim();

  if (name == "change") {
    int secondSpace = rest.indexOf(' ');
    if (secondSpace > 0) {
      String newText = rest.substring(secondSpace + 1);
      newText.trim();
      changeAddress(rest.substring(0, secondSpace), newText);
      return;
    }
  } else if (name == "factory" && rest.length() > 0) {
    factoryReset(rest);
    return;
  }

  Serial.println("ERROR invalid command");
  printHelp();
}

void setup() {
  Serial.begin(115200);
  // USB boards (ESP32-S3, RP2040/RP2350): wait up to 3 s for the Serial Monitor.
  while (!Serial && millis() < 3000UL) delay(10);
  delay(200);

  bus.beginRecovered(SDA_PIN, SCL_PIN);

  Serial.println("UE0177 change address");
  printHelp();
  scanBus();
}

void loop() {
  while (Serial.available()) {
    char character = (char)Serial.read();
    if (character == '\r' || character == '\n') {
      if (inputLine.length() > 0U) {
        processCommand(inputLine);
        inputLine = "";
      }
    } else if (inputLine.length() < 64U) {
      inputLine += character;
    } else {
      inputLine = "";
      Serial.println("ERROR command is too long");
    }
  }
}
