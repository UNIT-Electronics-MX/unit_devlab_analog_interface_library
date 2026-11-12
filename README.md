# DevLab_AnalogInterface

Arduino library for the **UE0177 Analog-Digital to I2C Sensor Interface** using
the DevLab Device Protocol (DDP) over I2C.

The module lets you add any analog sensor to an I2C bus. The signal enters at
`SIGIN` (JP2 pin 1), is read by a 12-bit ADC and reported over I2C. An optional
GPIO line (JP2 pin 3) can be configured as an input, a fixed output or an
output that follows an analog threshold.

## Features

- DDP 1.x master through the `DevLabDDP` dependency
- Device identification (Device ID `0x0119`, factory address `0x39`) in `begin()`
- 12-bit ADC count, voltage in mV or V for a 3.3 V or 5 V supply, percentage
- Software threshold with hysteresis, debounce and edge events
- GPIO configured from `begin()`: input, output low/high, or output that
  follows the threshold (normal or inverted)
- Bus recovery (`beginRecovered`) for a slave left mid-transaction
- Custom I2C pins

## Hardware

```
sensor --> SIGIN --+--> comparator vs R6 --> OUT --> J2 (digital output)
                   +--> buffer --> ADC --> I2C
GPIO --> JP2 pin 3
```

`OUT` is the hardware comparator with the R6 potentiometer threshold. It is
available on J2 and is **not** read over I2C. The software threshold of this
library is independent of R6. It starts at count 2300 on, 1800 off, and
`setThresholds()` changes it from the sketch. Use either JP2 or J2 as the
signal entry, never both.

## Usage

```cpp
#include <Wire.h>
#include <DevLab_AnalogInterface.h>

DevLab_AnalogInterface analog(Wire, DevLab_AnalogInterface::DEFAULT_ADDRESS, 100000UL);

void setup() {
  Serial.begin(115200);
  // Identify the node and make the GPIO follow the threshold.
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
| `begin(mode)`, `begin(sda, scl, mode)`, `beginRecovered(sda, scl, mode)` | Start the bus, identify the node and set the GPIO mode (default `Input`). Returns false if the node is missing or the mode failed; `isConnected()` tells which. |
| `readRaw(raw)` | 12-bit count, 0..4095 |
| `readMillivolts(mv)`, `readVolts(v)` | Voltage using `setSupplyMillivolts()` (default 3300) |
| `readPercentage(pct)` | Count as a percentage of full scale |
| `readAboveThreshold(above)` | Software threshold state |
| `readThresholds(high, low)` | Hysteresis levels in counts |
| `setThresholds(high, low)`, `setThresholdsMillivolts(highMv, lowMv)` | Changes the hysteresis levels |
| `resetThresholds()` | Back to the defaults (2300 / 1800) |
| `readStatus(status)`, `poll(state)` | Status byte; reading **clears** the edge event |
| `setGpioMode(mode)`, `readGpioMode(mode)` | GPIO mode |
| `gpioInput()`, `gpioWrite(level)`, `gpioFollowThreshold(inverted)` | GPIO shortcuts |
| `readGpio(level)` | GPIO level in any mode |
| `printInfo(out)`, `deviceInfo()`, `isConnected()`, `busReady()`, `address()` | Device information |

GPIO modes (`DevLab_AnalogGpioMode`): `Input` (pull-up), `OutputLow`,
`OutputHigh`, `FollowThreshold`, `FollowThresholdInverted`.

## Notes

- **The GPIO resets to input.** The mode is lost on every reset, and `begin()`
  always sends it. Call `begin()` again if the master restarts or the module
  loses power.
- **Supply voltage.** The ADC count is a fraction of the supply, so a 5 V board
  only needs `setSupplyMillivolts(5000)`. At 5 V the module needs a high level of
  about 3.5 V on the I2C lines, so a 3.3 V master is only reliable at a 3.3 V
  supply.
- **RP2040/RP2350 masters.** The I2C controller is fixed by the pin: SDA on
  GPIO 0, 4, 8, ... 24 is I2C0 (`Wire`) and on GPIO 2, 6, 10, ... is I2C1
  (`Wire1`). The examples choose the object from the SDA pin.
- **Address.** The I2C address is fixed (`0x39`). I2C runs at 100 kHz.

## Examples

See [examples/README.md](examples/README.md): analog reads (`adc/`), the GPIO as
input, output, threshold follower, host-driven alarm and a mode console
(`gpio/`), and an I2C scanner (`i2c/`).

## Installation

Copy or link this folder into the Arduino `libraries` folder as
`DevLab_AnalogInterface`. Install `DevLabDDP` and `DevLab_Interface` too.

## License

MIT, see [LICENSE](LICENSE).
