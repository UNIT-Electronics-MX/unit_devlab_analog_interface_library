# DevLab_AnalogInterface

Arduino library for the **UE0177 Analog-Digital to I2C Sensor Interface** using
the DevLab Device Protocol (DDP) over I2C.

The module lets you add any analog sensor to an I2C bus. The signal enters at
`SIGIN` (JP2 pin 1), a unity-gain buffer drives the 12-bit ADC of a PY32F003
(PA2), and the node reports the count over I2C. An optional GPIO line (PB5,
JP2 pin 3) can be configured as an input, a fixed output or an output that
follows an analog threshold.

The firmware is in `unit_firmware_i2c_analog_interface_py32` of
[unit_devlab_atom_i2c_main](https://github.com/UNIT-Electronics-Labs/unit_devlab_atom_i2c_main).
It needs firmware 0.2 or newer; `setThresholds()` needs 0.3.

> Status: **UNSTABLE**. The library compiles for ESP32-S3, Arduino UNO, RP2040
> and the Pulsar RP2350A. On a UE0177 board with a Pulsar RP2350A master it
> identifies the node and runs `i2c_scanner`, `readAnalog`, `gpioInput`,
> `gpioOutput` and `gpioFollowThreshold`. Other masters and a real sensor are
> not tested yet.

## Features

- DDP 1.x master through the `DevLabDDP` dependency
- Device identification (Device ID `0x0119`, factory address `0x39`) in `begin()`
- 12-bit ADC count, voltage in mV or V for a 3.3 V or 5 V supply, percentage
- Software threshold with hysteresis and debounce computed in the node, with
  edge events
- PB5 GPIO configured from `begin()`: input, output low/high, or output that
  follows the threshold (normal or inverted)
- Bus recovery (`beginRecovered`) for a slave left mid-transaction
- Custom I2C pins

## Hardware

```
sensor --> SIGIN --+--> U1A comparator vs R6 --> OUT --> J2 (digital output, no MCU)
                   +--> U1B buffer --> SIG --> PA2 (ADC) --> I2C
PB5 (GPIO) --> JP2 pin 3
```

`OUT` is the hardware comparator with the R6 potentiometer threshold. It is
available on J2 and is **not** read over I2C. The software threshold of this
library is independent of R6. It starts at count 2300 on, 1800 off, and
`setThresholds()` changes it from the sketch (kept in the node's RAM). Use either JP2 or J2 as the signal entry, never both.

## Usage

```cpp
#include <Wire.h>
#include <DevLab_AnalogInterface.h>

DevLab_AnalogInterface analog(Wire, DevLab_AnalogInterface::DEFAULT_ADDRESS, 100000UL);

void setup() {
  Serial.begin(115200);
  // Identify the node and make PB5 follow the threshold.
  if (!analog.beginRecovered(6, 7, DevLab_AnalogGpioMode::FollowThreshold)) {
    Serial.println("UE0177 not found");
    return;
  }
  analog.setSupplyMillivolts(3300);  // 5000 for a 5 V board
}

void loop() {
  uint16_t mv;
  if (analog.readMillivolts(mv)) Serial.println(mv);
  delay(100);
}
```

## API

| Method | Purpose |
|---|---|
| `begin(mode)`, `begin(sda, scl, mode)`, `beginRecovered(sda, scl, mode)` | Start the bus, identify the node and set the PB5 mode (default `Input`). Returns false if the node is missing or the mode failed; `isConnected()` tells which. |
| `readRaw(raw)` | 12-bit count, 0..4095 |
| `readMillivolts(mv)`, `readVolts(v)` | Voltage using `setSupplyMillivolts()` (default 3300) |
| `readPercentage(pct)` | Count as a percentage of full scale |
| `readAboveThreshold(above)` | Software threshold state |
| `readThresholds(high, low)` | Hysteresis levels in counts |
| `setThresholds(high, low)`, `setThresholdsMillivolts(highMv, lowMv)` | Changes the hysteresis levels (node RAM only, firmware 0.3 or newer) |
| `resetThresholds()` | Back to the firmware defaults (2300 / 1800) |
| `readStatus(status)`, `poll(state)` | Status byte; reading **clears** the edge event |
| `setGpioMode(mode)`, `readGpioMode(mode)` | PB5 mode |
| `gpioInput()`, `gpioWrite(level)`, `gpioFollowThreshold(inverted)` | PB5 shortcuts |
| `readGpio(level)` | PB5 level in any mode |
| `printInfo(out)`, `deviceInfo()`, `isConnected()`, `busReady()`, `address()` | Device information |

PB5 modes (`DevLab_AnalogGpioMode`): `Input` (pull-up), `OutputLow`,
`OutputHigh`, `FollowThreshold`, `FollowThresholdInverted`.

## Notes

- **PB5 resets to input.** The node forgets the mode on every reset, and
  `begin()` always sends it. Call `begin()` again if the master restarts or the
  module loses power.
- **PB5 is JP2 pin 3.** An output mode drives the pin push-pull; do not combine
  it with a sensor that drives the same line.
- **Supply voltage.** The ADC count is a fraction of the supply, so a 5 V board
  only needs `setSupplyMillivolts(5000)`. At 5 V the PY32 I2C inputs need a high
  level of about 3.5 V, so a 3.3 V master is only reliable at a 3.3 V supply.
- **RP2040/RP2350 masters.** The I2C controller is fixed by the pin: SDA on
  GPIO 0, 4, 8, ... 24 is I2C0 (`Wire`) and on GPIO 2, 6, 10, ... is I2C1
  (`Wire1`). The examples choose the object from the SDA pin. On the Pulsar,
  GPIO 24/25 are I2C0, so `Wire1` (which the board core maps to those pins)
  never reaches the bus and every transfer times out.
- **Address.** The firmware cannot change its I2C address, so there is no
  address-change example. I2C runs at 100 kHz.

## Examples

See [examples/README.md](examples/README.md): analog reads (`adc/`), PB5 as
input, output, threshold follower, host-driven alarm and a mode console
(`gpio/`), and an I2C scanner (`i2c/`).

## Installation

Copy or link this folder into the Arduino `libraries` folder as
`DevLab_AnalogInterface`. Install `DevLabDDP` and `DevLab_Interface` too.

## License

MIT, see [LICENSE](LICENSE).
